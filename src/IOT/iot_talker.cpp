#include "ros/ros.h"
#include "beginning_with_ros/IOTsensor.h"
#include <stdio.h>
#include <random>

int main (int argc, char **argv) {
    ros::init(argc, argv, "iot_talker");
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<beginning_with_ros::IOTsensor>("iot_info", 10);
    ros::Rate rate(1);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_temp(0, 0.5);
    std::uniform_real_distribution<> rand_hum(0, 0.05);

    while (ros::ok()) {
         beginning_with_ros::IOTsensor message;
         message.name = "IOT";
         message.id = 1;
         message.temperature = 24.5F + rand_temp(gen);
         message.humidity = 0.75F + rand_hum(gen);

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