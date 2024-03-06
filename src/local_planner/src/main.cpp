#include <ros/ros.h>
#include <transformer.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "transformer");
    ros::NodeHandle nh;

    TfRemap transformer(nh);

    ros::spin();
    // while (ros::ok()) {
    //     ros::spinOnce();
    // }
    return 0;

}