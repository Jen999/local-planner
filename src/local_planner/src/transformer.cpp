#include <transformer.hpp>

TfRemap::TfRemap(ros::NodeHandle &nh) : node_handle(nh),
    odom_buffer(ros::Duration(10.0)), velo_buffer(ros::Duration(10.0)) {

    subOdom = node_handle.subscribe<nav_msgs::Odometry> ("/odom", 5, &TfRemap::odomHandler, this);
    subVelodyne = node_handle.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 5, &TfRemap::velodyneHandler, this);

    pubStateEstimation = node_handle.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
    pubRegisteredScan = node_handle.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);

    odom_buffer.setUsingDedicatedThread(true);
    velo_buffer.setUsingDedicatedThread(true);

    odom_listener.reset(new tf2_ros::TransformListener(odom_buffer));
    velo_listener.reset(new tf2_ros::TransformListener(velo_buffer));
    ROS_INFO_STREAM_THROTTLE(1, "Initialize Transform Listeners");
}

void TfRemap::odomHandler(const nav_msgs::Odometry::ConstPtr& odom) {
    nav_msgs::Odometry stateEstimation;
    nav_msgs::Odometry odomData = *odom;

    geometry_msgs::TransformStamped transformOdom;
    try {
        transformOdom = odom_buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
        ROS_INFO_STREAM_THROTTLE(1, "Transform frame odom to map exists");
        ROS_INFO_STREAM_THROTTLE(1, "Odom translation:" << " x: " << transformOdom.transform.translation.x 
                                                        << " y: " << transformOdom.transform.translation.y 
                                                        << " z: " << transformOdom.transform.translation.z << "\n"
                                << "Odom rotation:" << " x: " << transformOdom.transform.rotation.x
                                                    << " y: " << transformOdom.transform.rotation.y
                                                    << " z: " << transformOdom.transform.rotation.z
                                                    << " w: " << transformOdom.transform.rotation.w);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Odom Transform Error: %s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // Transform Pose
    geometry_msgs::Pose mapPose;
    try {
        tf2::doTransform(odomData.pose.pose, stateEstimation.pose.pose, transformOdom);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to transform odom pose: %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // Retain PoseCovariance
    stateEstimation.pose.covariance = odomData.pose.covariance;
    // Retain TwistWithCovariance
    stateEstimation.twist.twist = odomData.twist.twist;
    stateEstimation.twist.covariance = odomData.twist.covariance;

    stateEstimation.header.stamp = ros::Time::now();
    stateEstimation.header.frame_id = "map";
    stateEstimation.child_frame_id = "robot_footprint";
    ROS_INFO_STREAM_THROTTLE(1, "Transformed from 'odom' to 'map'");

    pubStateEstimation.publish(stateEstimation);
}

void TfRemap::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr& velo) {
    sensor_msgs::PointCloud2 registeredScan;
    sensor_msgs::PointCloud2 veloData = *velo;

    geometry_msgs::TransformStamped transformVelo;
    try {
        transformVelo = velo_buffer.lookupTransform("map", "velodyne", ros::Time(0));
        ROS_INFO_STREAM_THROTTLE(1, "Transform frame velo to map exists");
        ROS_INFO_STREAM_THROTTLE(1, "Velo translation:" << " x: " << transformVelo.transform.translation.x 
                                                        << " y: " << transformVelo.transform.translation.y 
                                                        << " z: " << transformVelo.transform.translation.z << "\n"
                                << "Velo rotation:" << " x: " << transformVelo.transform.rotation.x
                                                    << " y: " << transformVelo.transform.rotation.y
                                                    << " z: " << transformVelo.transform.rotation.z
                                                    << " w: " << transformVelo.transform.rotation.w);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Velo Transform Error: %s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    // Transform PointCloud2
    try {
        tf2::doTransform(veloData, registeredScan, transformVelo);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to transform velo pointcloud: %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    registeredScan.header.stamp = ros::Time::now();
    registeredScan.header.frame_id = "map";
    ROS_INFO_STREAM_THROTTLE(1, "Transformed from 'velodyne' to 'map'");

    pubRegisteredScan.publish(registeredScan);
}

// Errors to fix:
// [ERROR] [1709715790.627878738, 238.167000000]: Client [/gazebo] wants topic /cmd_vel to have datatype/md5sum [geometry_msgs/Twist/9f195f881246fdfa2798d1d3eebca84a], but our version has [geometry_msgs/TwistStamped/98d34b0043a2093cf9d9345ab6eef12e]. Dropping connection.
// OptimizerG2O.cpp:1214::optimize() Computing marginals: vertex 44 has negative hessian index (-1). Cannot compute last pose covariance.
// Rtabmap.cpp:2862::process() Rejected loop closure 104 -> 128: Not enough features in images (old=0, new=33, min=15)


