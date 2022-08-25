#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <stdio.h>

void callback_function(const turtlesim::Pose::ConstPtr &message) {
    ROS_INFO("");
    printf("    x: %f \n", message->x);
    printf("    y: %f \n", message->y);
    printf("    theta: %f \n", message->theta);
    printf("\n");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "location");

    ros::NodeHandle nodeHandler;

    ros::Subscriber subscriber = nodeHandler.subscribe("turtle1/pose", 1, callback_function);

    ros::spin();

    return 0;
}