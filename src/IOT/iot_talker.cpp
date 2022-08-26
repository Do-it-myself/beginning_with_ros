#include "ros/ros.h"
#include "beginning_with_ros/IOTsensor.h"
#include <stdio.h>


int main (int argc, char **argv) {
    ros::init(argc, argv, "iot_talker");
    
    ros::NodeHandle nodeHandle;

    ros::Publisher publisher = nodeHandle.advertise<beginning_with_ros::IOTsensor>("iot_info", 10);

    ros::Rate rate(1);

    while (ros::ok()) {
         beginning_with_ros::IOTsensor message;
         message.name = "IOT";
         message.id = 1;
         message.temperature = 25.0F;
         message.humidity = 0.8F;

         ROS_INFO("");
         printf("   Name: %s\n", message.name.c_str());
         printf("   ID: %d\n", message.id);
         printf("   Temperature: %f\n", message.temperature);
         printf("   Humidity: %f\n", message.humidity);

         publisher.publish(message);
         ros::spinOnce();

         rate.sleep();
    }
}