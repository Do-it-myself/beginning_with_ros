#include "ros/ros.h"
#include "beginning_with_ros/AddTwoInts.h"
#include <stdio.h>

bool add_handler(beginning_with_ros::AddTwoInts::Request &request,
                 beginning_with_ros::AddTwoInts::Response &response)

{
    response.sum = request.a + request.b;

    ROS_INFO("");    
    printf("    Request from client: %ld, %ld\n", (long int)request.a, (long int)request.b);
    printf("    Response: %ld\n", (long int)response.sum);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_server_cpp");
    ros::NodeHandle nodeHandle;
    ros::ServiceServer server = nodeHandle.advertiseService("add_two_ints", add_handler);
    ROS_INFO("Server ready");
    ros::spin();

    return 0;
}