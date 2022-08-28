#include "ros/ros.h"
#include "std_msgs/String.h"

void callback_function(const std_msgs::String::ConstPtr &message) {
    ROS_INFO("CPP - %s", message->data.c_str());
}

int main(int argc, char **argv) {
    /* ----- NODE ----- */
    // Create and initialize node
    ros::init(argc, argv, "listener_cpp");

    // Create node handler
    ros::NodeHandle nodeHandler;

    /* ----- SUBSCRIBER ----- */
    // Create subscriber
    ros::Subscriber subscriber = nodeHandler.subscribe("test_talk", 10, callback_function);

    // Enter listening mode 
    ros::spin();

    return 0;
}