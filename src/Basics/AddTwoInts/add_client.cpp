#include "ros/ros.h"
#include "beginning_with_ros/AddTwoInts.h"
#include <stdio.h>
#include <stdlib.h>

int main (int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "add_client_cpp");
    ros::NodeHandle nodeHandle;

    // initialize client
    ros::ServiceClient client = nodeHandle.serviceClient<beginning_with_ros::AddTwoInts>("add_two_ints");
    
    // check argument number
    if (argc == 3) {
        beginning_with_ros::AddTwoInts srv;
        srv.request.a = atoll(argv[1]);
        srv.request.b = atoll(argv[2]);

        // check if client has called for service successfully
        // feed srv (with .request.a & .request.b, get back .response.sum)
        if (client.call(srv)) {
            ROS_INFO("");
            printf("    Request: %ld, %ld\n", (long int)srv.request.a, (long int)srv.request.b);
            printf("    Response from server: %ld\n", (long int)srv.response.sum);
        }

        else {
            ROS_ERROR("Service call failed");

            return 1;
        }
    }

    else {
        ROS_INFO("Incorrect number of arguments - Need: x y");

        return 1;
    }
    
    return 0;
}