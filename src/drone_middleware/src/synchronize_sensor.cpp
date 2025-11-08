// sensor_synchronizer_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

class SensorSynchronizer
{
public:
    SensorSynchronizer() : pnh_("~")
    {
        // params (private)
        pnh_.param<std::string>("lidar_topic", lidar_topic_, "/rslidar_merged/points");
        pnh_.param<std::string>("imu_topic", imu_topic_, "/imu");
        pnh_.param<std::string>("camera_topic", camera_topic_, "/camera/image_color");

        pnh_.param<std::string>("lidar_output_topic", lidar_output_topic_, "/synced/lidar");
        pnh_.param<std::string>("imu_output_topic", imu_output_topic_, "/synced/imu");
        pnh_.param<std::string>("camera_output_topic", camera_output_topic_, "/synced/camera");

        pnh_.param<int>("queue_size", queue_size_, 50);
        pnh_.param<double>("slop", slop_, 0.1);  // 100ms tolerance (tune later)

        // Use global NodeHandle for actual subscriptions/publishers
        ros::NodeHandle nh;

        // Publishers
        lidar_pub_  = nh.advertise<sensor_msgs::PointCloud2>(lidar_output_topic_, 10);
        imu_pub_    = nh.advertise<sensor_msgs::Imu>(imu_output_topic_, 10);
        camera_pub_ = nh.advertise<sensor_msgs::Image>(camera_output_topic_, 10);

        // Subscribers
        lidar_sub_.subscribe(nh, lidar_topic_, queue_size_);
        imu_sub_.subscribe(nh, imu_topic_, queue_size_);

        // image_transport subscriber filter (supports compressed/raw transports)
        image_transport::ImageTransport it(nh);
        camera_sub_.reset(new image_transport::SubscriberFilter());
        camera_sub_->subscribe(it, camera_topic_, queue_size_);

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2, sensor_msgs::Imu, sensor_msgs::Image
        > SyncPolicy;

        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(queue_size_), lidar_sub_, imu_sub_, *camera_sub_));
        sync_->setMaxIntervalDuration(ros::Duration(slop_));
        sync_->registerCallback(boost::bind(&SensorSynchronizer::syncCallback, this, _1, _2, _3));

        // small diagnostics (counts)
        last_report_ = ros::Time::now();
        ROS_INFO("SensorSynchronizer started. Subscribing to: %s, %s, %s",
                 lidar_topic_.c_str(), imu_topic_.c_str(), camera_topic_.c_str());
    }

    void syncCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg,
                      const sensor_msgs::Imu::ConstPtr& imu_msg,
                      const sensor_msgs::Image::ConstPtr& camera_msg)
    {
        ++sync_count_;
        double t_l = lidar_msg->header.stamp.toSec();
        double t_i = imu_msg->header.stamp.toSec();
        double t_c = camera_msg->header.stamp.toSec();
        double li = fabs(t_l - t_i) * 1000.0;
        double lc = fabs(t_l - t_c) * 1000.0;

        if (sync_count_ % 20 == 0) {
            ROS_INFO("Sync #%d LiDAR-IMU=%.2fms LiDAR-CAM=%.2fms", sync_count_, li, lc);
        }

        lidar_pub_.publish(lidar_msg);
        imu_pub_.publish(imu_msg);
        camera_pub_.publish(camera_msg);
    }

private:
    ros::NodeHandle pnh_;

    // subs
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    std::shared_ptr<image_transport::SubscriberFilter> camera_sub_;

    // pubs
    ros::Publisher lidar_pub_, imu_pub_, camera_pub_;

    // sync
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::PointCloud2, sensor_msgs::Imu, sensor_msgs::Image
    > SyncPolicy;
    boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;

    // params
    std::string lidar_topic_, imu_topic_, camera_topic_;
    std::string lidar_output_topic_, imu_output_topic_, camera_output_topic_;
    int queue_size_;
    double slop_;

    // stats
    int sync_count_ = 0;
    ros::Time last_report_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_synchronizer");
    SensorSynchronizer ss;
    ros::spin();
    return 0;
}

