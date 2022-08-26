#include "ros/ros.h"
#include "Beginning-with-ROS/IOTsensor.h"


int main (int argc, char **argv) {
    ros::init(argc, argv, "iot");
    
    ros::NodeHandle nodeHandle;

    ros::Publisher publisher = nodeHandle.advertise<Beginning-with-ROS::IOTsensor>("iot_info", 10)

    ros::Rate rate(1);

    while (ros::ok()) {
         Beginning-with-ROS::
    }
}