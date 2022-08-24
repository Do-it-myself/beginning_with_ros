#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{

    /* ----- NODE ----- */
    // Create and initialize node
    ros::init(argc, argv, "talker_cpp");

    // Create node handle
    ros::NodeHandle nodeHandle; // EXTRA

    /* ----- PUBLISHER ----- */
    // Create publisher
    // State type; arg: node_name, queue_size
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("test_talk", 10);

    // Define publication rate
    ros::Rate rate(1);

    int i = 0;
    while (ros::ok())
    {
        std::stringstream ss;
        ss << "Count: " << i;
        std_msgs::String message;
        message.data = ss.str();

        // Put string to log
        ROS_INFO("CPP - %s\n", message.data.c_str());

        // Publish message
        publisher.publish(message);
        ros::spinOnce(); // EXTRA

        // Control message sending rate
        rate.sleep();

        i++;
    }

    return 0;
}