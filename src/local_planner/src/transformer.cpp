#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

nav_msgs::Odometry odomData;
sensor_msgs::PointCloud2 veloData;

void odomHandler(const nav_msgs::Odometry::ConstPtr& odom) {
    odomData = *odom;
}

void velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr& velo) {
    veloData = *velo;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "transformer");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry> ("/odom", 5, odomHandler);
    ros::Subscriber subVelodyne = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 5, velodyneHandler);

    ros::Publisher pubStateEstimation = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
    ros::Publisher pubRegisteredScan = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);

    tf2_ros::Buffer odom_buffer;
    tf2_ros::TransformListener odom_listener(odom_buffer);

    tf2_ros::Buffer velo_buffer;
    tf2_ros::TransformListener velo_listener(velo_buffer);

    while (ros::ok()) {
        ros::spinOnce();

        geometry_msgs::TransformStamped transformOdom;
        nav_msgs::Odometry stateEstimation;
        try {
            transformOdom = odom_buffer.lookupTransform("map", "odom", ros::Time(0));
            // tf2::doTransform(odomData, stateEstimation, transformOdom);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("OdomData Transform: %s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        stateEstimation.header.frame_id = "map";
        stateEstimation.child_frame_id = "robot_footprint";

        stateEstimation.header.stamp = ros::Time::now();
        stateEstimation.pose.pose.orientation.x = transformOdom.transform.rotation.x + odomData.pose.pose.orientation.x;
        stateEstimation.pose.pose.orientation.y = transformOdom.transform.rotation.y + odomData.pose.pose.orientation.y;
        stateEstimation.pose.pose.orientation.z = transformOdom.transform.rotation.z + odomData.pose.pose.orientation.z;
        stateEstimation.pose.pose.orientation.w = transformOdom.transform.rotation.w + odomData.pose.pose.orientation.w;
        stateEstimation.pose.pose.position.x = transformOdom.transform.translation.x + odomData.pose.pose.position.x;
        stateEstimation.pose.pose.position.y = transformOdom.transform.translation.y + odomData.pose.pose.position.y;
        stateEstimation.pose.pose.position.z = transformOdom.transform.translation.z + odomData.pose.pose.position.z;
        stateEstimation.twist.twist.angular.x = odomData.twist.twist.angular.x;
        stateEstimation.twist.twist.angular.y = odomData.twist.twist.angular.y;
        stateEstimation.twist.twist.angular.z = odomData.twist.twist.angular.z;
        stateEstimation.twist.twist.linear.x = odomData.twist.twist.linear.x;
        stateEstimation.twist.twist.linear.y = odomData.twist.twist.linear.y;
        stateEstimation.twist.twist.linear.z = odomData.twist.twist.linear.z;
        pubStateEstimation.publish(stateEstimation);

        geometry_msgs::TransformStamped transformVelo;
        sensor_msgs::PointCloud2 registeredScan;
        try {
            transformOdom = odom_buffer.lookupTransform("map", "velodyne", ros::Time(0));
            tf2::doTransform(veloData, registeredScan, transformVelo);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("VeloData Transform: %s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        registeredScan.header.frame_id = "map";
        pubRegisteredScan.publish(registeredScan);


    }


}