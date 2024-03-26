#include <ros/ros.h>
#include "dynamixel_pan_tilt_driver/pan_tilt_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pan_tilt_node");
    ros::NodeHandle priv_nh("~");
    PanTiltController panTiltController(priv_nh);

    ROS_INFO_NAMED("PanTiltDriver", "Pan Tilt Node Started");
    ros::spin();
    return 0;
}