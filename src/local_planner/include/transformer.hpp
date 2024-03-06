#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

class TfRemap {
    public:
        TfRemap(ros::NodeHandle &nh);

    private:
        void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);
        void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr& velo);
        ros::NodeHandle node_handle;
        ros::Subscriber subOdom;
        ros::Subscriber subVelodyne;
        ros::Publisher pubStateEstimation;
        ros::Publisher pubRegisteredScan;
        tf2_ros::Buffer odom_buffer;
        tf2_ros::Buffer velo_buffer;
        std::unique_ptr<tf2_ros::TransformListener> odom_listener;
        std::unique_ptr<tf2_ros::TransformListener> velo_listener;


};