#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <cctype>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

#include "cnpy.h"  // header-only NPY reader (vendor in third_party/cnpy)

class FisheyeDewarpNode {
public:
  FisheyeDewarpNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), it_(nh)
  {
    // --- Parameters ---
    pnh.param<std::string>("device", device_, "/dev/video4");
    pnh.param<int>("width", req_w_, 1920);
    pnh.param<int>("height", req_h_, 1080);
    pnh.param<double>("fps", fps_, 30.0);
    pnh.param<double>("balance", balance_, 0.0);
    pnh.param<std::string>("matrix_path", matrix_path_, std::string("camera_matrix.npy"));
    pnh.param<std::string>("dist_path", dist_path_, std::string("dist_coeffs.npy"));
    pnh.param<bool>("publish_side_by_side", publish_sbs_, true);
    pnh.param<std::string>("frame_id", frame_id_, std::string("camera"));  // <-- Option A

    // Topics
    std::string ns;
    pnh.param<std::string>("image_namespace", ns, std::string(""));
    std::string topic_orig   = ns.empty() ? "image_raw"      : ns + "/image_raw";
    std::string topic_dewarp = ns.empty() ? "image_dewarped" : ns + "/image_dewarped";
    std::string topic_sbs    = ns.empty() ? "image_sbs"      : ns + "/image_sbs";

    pub_orig_   = it_.advertise(topic_orig,   1);
    pub_dewarp_ = it_.advertise(topic_dewarp, 1);
    if (publish_sbs_) pub_sbs_ = it_.advertise(topic_sbs, 1);

    // Live balance adjustment
    sub_balance_ = nh_.subscribe<std_msgs::Float64>("fisheye_dewarp/set_balance", 1,
      [this](const std_msgs::Float64::ConstPtr& msg){
        double b = std::min(1.0, std::max(0.0, msg->data));
        if (std::abs(b - balance_) > 1e-6) {
          balance_ = b;
          ROS_INFO("balance -> %.2f (recompute maps)", balance_);
          if (have_size_) recomputeMaps();
        }
      });

    // Load calibration
    loadCalibration(matrix_path_, dist_path_);

    // Open camera
    openCamera();

    // First frame => actual size
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      throw std::runtime_error("Failed to read first frame from camera.");
    }
    size_ = frame.size();
    have_size_ = true;

    // Precompute maps
    recomputeMaps();

    ROS_INFO("fisheye_dewarp_node: %s %dx%d @ %.1f FPS, balance=%.2f, frame_id=%s",
             device_.c_str(), size_.width, size_.height, fps_, balance_, frame_id_.c_str());
  }

  void spin() {
    ros::Rate rate(std::max(1.0, fps_));
    while (ros::ok()) {
      cv::Mat frame;
      if (!cap_.read(frame) || frame.empty()) {
        ROS_WARN_THROTTLE(2.0, "Frame grab failed");
        ros::spinOnce();
        rate.sleep();
        continue;
      }

      // Common header for all images
      std_msgs::Header hdr;
      hdr.stamp = ros::Time::now();
      hdr.frame_id = frame_id_;  // <-- set frame_id

      // Publish original
      sensor_msgs::ImagePtr msg_orig = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub_orig_.publish(msg_orig);

      // Dewarp
      cv::Mat undist;
      cv::remap(frame, undist, map1_, map2_, cv::INTER_LINEAR);

      sensor_msgs::ImagePtr msg_dew = cv_bridge::CvImage(hdr, "bgr8", undist).toImageMsg();
      pub_dewarp_.publish(msg_dew);

      if (publish_sbs_ && pub_sbs_.getNumSubscribers() > 0) {
        int h = frame.rows, w = frame.cols;
        cv::Mat combo(h, w*2, frame.type());
        frame.copyTo(combo(cv::Rect(0, 0, w, h)));
        undist.copyTo(combo(cv::Rect(w, 0, w, h)));
        sensor_msgs::ImagePtr msg_sbs = cv_bridge::CvImage(hdr, "bgr8", combo).toImageMsg();
        pub_sbs_.publish(msg_sbs);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  static cv::Mat loadNPY(const std::string& path) {
    cnpy::NpyArray arr = cnpy::npy_load(path);
    const std::vector<size_t>& shp = arr.shape;
    if (arr.word_size != 8 && arr.word_size != 4)
      throw std::runtime_error("Only float32/float64 supported in " + path);

    int rows = 1, cols = 1;
    if (shp.size() == 2) {
      rows = static_cast<int>(shp[0]);
      cols = static_cast<int>(shp[1]);
    } else if (shp.size() == 1) {
      rows = static_cast<int>(shp[0]);
      cols = 1;
    } else {
      throw std::runtime_error("Unexpected NPY shape in " + path);
    }

    cv::Mat m(rows, cols, CV_64F);
    if (arr.word_size == 8) {
      const double* p = arr.data<double>();
      for (int r=0; r<rows; ++r)
        for (int c=0; c<cols; ++c) m.at<double>(r,c) = p[r*cols+c];
    } else {
      const float* p = arr.data<float>();
      for (int r=0; r<rows; ++r)
        for (int c=0; c<cols; ++c) m.at<double>(r,c) = static_cast<double>(p[r*cols+c]);
    }
    return m;
  }

  void loadCalibration(const std::string& Kp, const std::string& Dp) {
    K_ = loadNPY(Kp);
    D_ = loadNPY(Dp);
    if (K_.rows != 3 || K_.cols != 3)
      throw std::runtime_error("camera_matrix.npy must be 3x3");
    // Fisheye supports 4/8/12/14; proceed even if different shape
    if (!((D_.rows == 4 && D_.cols == 1) || (D_.rows == 1 && D_.cols == 4) ||
          D_.rows == 8 || D_.rows == 12 || D_.rows == 14)) {
      ROS_WARN("Unexpected dist_coeffs shape %dx%d; continuing", D_.rows, D_.cols);
    }
  }

  void openCamera() {
    bool numeric = !device_.empty() &&
                   std::all_of(device_.begin(), device_.end(), ::isdigit);
    if (numeric) {
      cap_.open(std::stoi(device_));
    } else {
      cap_.open(device_, cv::CAP_V4L2);
    }
    if (!cap_.isOpened()) throw std::runtime_error("Could not open camera: " + device_);

    if (req_w_ > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, req_w_);
    if (req_h_ > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, req_h_);
    if (fps_   > 0) cap_.set(cv::CAP_PROP_FPS, fps_);
    // MJPG often helps at high res/FPS; comment out if your camera dislikes it
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
  }

  void recomputeMaps() {
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      K_, D_, size_, R, Knew_, balance_);
    cv::fisheye::initUndistortRectifyMap(
      K_, D_, R, Knew_, size_, CV_16SC2, map1_, map2_);
  }

  // Members
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_orig_, pub_dewarp_, pub_sbs_;
  ros::Subscriber sub_balance_;

  std::string device_, matrix_path_, dist_path_;
  int req_w_{1920}, req_h_{1080};
  double fps_{30.0}, balance_{0.0};
  bool publish_sbs_{true};
  bool have_size_{false};
  std::string frame_id_{"camera"};  // <-- Option A member

  cv::VideoCapture cap_;
  cv::Size size_;
  cv::Mat K_, D_, Knew_;
  cv::Mat map1_, map2_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fisheye_dewarp_node");
  ros::NodeHandle nh, pnh("~");
  try {
    FisheyeDewarpNode node(nh, pnh);
    node.spin();
  } catch (const std::exception& e) {
    ROS_FATAL("Fatal: %s", e.what());
    return 1;
  }
  return 0;
}
