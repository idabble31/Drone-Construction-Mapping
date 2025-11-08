// sensor_synchronizer_with_imu_interp.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <deque>
#include <mutex>
#include <Eigen/Geometry> // for quaternion slerp

class SyncWithImuInterp {
public:
    SyncWithImuInterp()
    : pnh_("~")
    {
        pnh_.param<std::string>("lidar_topic", lidar_topic_, "/rslidar_merged/points");
        pnh_.param<std::string>("imu_topic", imu_topic_, "/imu");
        pnh_.param<std::string>("camera_topic", camera_topic_, "/camera/image_color");

        pnh_.param<std::string>("lidar_output", out_lidar_, "/synced/lidar");
        pnh_.param<std::string>("imu_output", out_imu_, "/synced/imu");
        pnh_.param<std::string>("cam_output", out_camera_, "/synced/camera");

        pnh_.param<int>("queue_size", queue_size_, 50);
        pnh_.param<double>("slop", slop_, 0.05);

        ros::NodeHandle nh;

        // publishers
        pub_lidar_ = nh.advertise<sensor_msgs::PointCloud2>(out_lidar_, 5);
        pub_imu_   = nh.advertise<sensor_msgs::Imu>(out_imu_, 10);
        pub_cam_   = nh.advertise<sensor_msgs::Image>(out_camera_, 5);

        // subscribers
        lidar_sub_.subscribe(nh, lidar_topic_, 10);
        imu_sub_raw_ = nh.subscribe(imu_topic_, 200, &SyncWithImuInterp::imuCallback, this);

        image_transport::ImageTransport it(nh);
        cam_sub_.reset(new image_transport::SubscriberFilter());
        cam_sub_->subscribe(it, camera_topic_, 10);

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2, sensor_msgs::Image> LidarCamPolicy;
        sync_lc_.reset(new message_filters::Synchronizer<LidarCamPolicy>(LidarCamPolicy(queue_size_), lidar_sub_, *cam_sub_));
        sync_lc_->setMaxIntervalDuration(ros::Duration(slop_));
        sync_lc_->registerCallback(boost::bind(&SyncWithImuInterp::lidarCameraCallback, this, _1, _2));

        ROS_INFO("SyncWithImuInterp started");
    }

private:
    // IMU buffer & callback
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        // keep a short history: 1s or a fixed sample count
        imu_buffer_.push_back(imu);
        while (imu_buffer_.size() > 500) imu_buffer_.pop_front(); // guard size
        // also drop too-old samples
        ros::Time cutoff = imu->header.stamp - ros::Duration(1.0);
        while (!imu_buffer_.empty() && imu_buffer_.front()->header.stamp < cutoff) imu_buffer_.pop_front();
    }

    // called when lidar+camera are synchronized
    void lidarCameraCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar,
                             const sensor_msgs::Image::ConstPtr& cam)
    {
        // choose a reference timestamp (you can pick lidar->header.stamp or cam->header.stamp; choose mid/average if needed)
        ros::Time target_t = cam->header.stamp;

        sensor_msgs::Imu interpolated;
        bool ok = interpolateImu(target_t, interpolated);
        if (!ok) {
            ROS_WARN_THROTTLE(5.0, "Failed to interpolate IMU for time %.6f", target_t.toSec());
            return;
        }

        // publish synced messages
        pub_lidar_.publish(lidar);
        pub_cam_.publish(cam);

        // set the interpolated header stamp to target
        interpolated.header.stamp = target_t;
        // optionally set frame_id to IMU frame or unify
        pub_imu_.publish(interpolated);
    }

    // find two IMU samples around target and interpolate
    bool interpolateImu(const ros::Time& t, sensor_msgs::Imu& out) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imu_buffer_.size() < 2) return false;

        // find two samples i0 <= t <= i1
        size_t idx = 0;
        while (idx + 1 < imu_buffer_.size() && imu_buffer_[idx+1]->header.stamp <= t) ++idx;
        if (idx + 1 >= imu_buffer_.size()) return false; // need newer sample
        auto i0 = imu_buffer_[idx];
        auto i1 = imu_buffer_[idx+1];

        double t0 = i0->header.stamp.toSec();
        double t1 = i1->header.stamp.toSec();
        if (t1 <= t0) return false;

        double alpha = (t.toSec() - t0) / (t1 - t0);
        if (alpha < 0.0) alpha = 0.0;
        if (alpha > 1.0) alpha = 1.0;

        // linear interp for angular velocity & linear acceleration
        out = *i0; // copy baseline
        out.header.stamp = t; // set to target (publisher will override if you want)
        for (int k = 0; k < 3; ++k) {
            out.angular_velocity.x = (1.0 - alpha) * i0->angular_velocity.x + alpha * i1->angular_velocity.x;
            out.angular_velocity.y = (1.0 - alpha) * i0->angular_velocity.y + alpha * i1->angular_velocity.y;
            out.angular_velocity.z = (1.0 - alpha) * i0->angular_velocity.z + alpha * i1->angular_velocity.z;

            out.linear_acceleration.x = (1.0 - alpha) * i0->linear_acceleration.x + alpha * i1->linear_acceleration.x;
            out.linear_acceleration.y = (1.0 - alpha) * i0->linear_acceleration.y + alpha * i1->linear_acceleration.y;
            out.linear_acceleration.z = (1.0 - alpha) * i0->linear_acceleration.z + alpha * i1->linear_acceleration.z;
        }

        // slerp orientation using Eigen
        Eigen::Quaterniond q0(i0->orientation.w, i0->orientation.x, i0->orientation.y, i0->orientation.z);
        Eigen::Quaterniond q1(i1->orientation.w, i1->orientation.x, i1->orientation.y, i1->orientation.z);
        Eigen::Quaterniond qinterp = q0.slerp(alpha, q1);
        qinterp.normalize();
        out.orientation.x = qinterp.x();
        out.orientation.y = qinterp.y();
        out.orientation.z = qinterp.z();
        out.orientation.w = qinterp.w();

        // covariances: simple linear interpolation (or copy nearest)
        for (int i=0;i<9;i++){
            out.orientation_covariance[i] = (1.0-alpha)*i0->orientation_covariance[i] + alpha*i1->orientation_covariance[i];
            out.angular_velocity_covariance[i] = (1.0-alpha)*i0->angular_velocity_covariance[i] + alpha*i1->angular_velocity_covariance[i];
            out.linear_acceleration_covariance[i] = (1.0-alpha)*i0->linear_acceleration_covariance[i] + alpha*i1->linear_acceleration_covariance[i];
        }

        return true;
    }

    // members
    ros::NodeHandle pnh_;

    std::string lidar_topic_, imu_topic_, camera_topic_;
    std::string out_lidar_, out_imu_, out_camera_;
    int queue_size_;
    double slop_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    std::shared_ptr<image_transport::SubscriberFilter> cam_sub_;
    boost::shared_ptr< message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> > > sync_lc_;

    ros::Subscriber imu_sub_raw_;

    ros::Publisher pub_lidar_, pub_imu_, pub_cam_;

    std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    std::mutex imu_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_with_imu_interp");
    SyncWithImuInterp node;
    ros::spin();
    return 0;
}
