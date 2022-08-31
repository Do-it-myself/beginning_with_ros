#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <stdio.h>
#include <stdlib.h> // abs
#include <math.h> // sqrt
#include <cmath> // pow

// global objects/variables declaration
ros::Publisher publisher;
ros::Subscriber subscriber;
turtlesim::Pose pose_initial;
turtlesim::Pose pose_current;

// global functions declaration
void poseCallBack (const turtlesim::Pose::ConstPtr &message);
void move(double speed, double distance, bool isForward);

int main (int argc, char **argv) {
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle nodeHandle;

    publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    subscriber = nodeHandle.subscribe("turtle1/pose", 1, poseCallBack);

    move(1.0, 1.0, true);

    return 0;
}

void poseCallBack (const turtlesim::Pose::ConstPtr &message) {
    pose_current.x = message->x;
    pose_current.y = message->y;
    pose_current.theta = message->theta;
}

void move(double speed, double distance, bool isForward=true) {
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);

    // decide velocity
    if (isForward) {
        velocity.linear.x = abs(speed);
    }
    else {
        velocity.linear.x = -abs(speed);
    }
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    // store initial pose
    do {
        pose_initial = pose_current;
        ros::spinOnce();
        rate.sleep();
    } while(pose_initial.x == 0);
    
    double distance_current = 0;
    do {
        // check distance
        distance_current = abs(sqrt(pow((pose_current.x - pose_initial.x), 2) + pow((pose_current.y - pose_initial.y), 2)));

        // log
        ROS_INFO("Distance: %lf", distance_current);

        // publish velocity (move)
        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();

    } while (distance_current < distance);

    // log
    ROS_INFO("Distance: %lf", distance_current);
    ROS_INFO("Reached");
    
    // publish velocity (stop)
    velocity.linear.x = 0;
    publisher.publish(velocity);
    ros::spinOnce();
}





