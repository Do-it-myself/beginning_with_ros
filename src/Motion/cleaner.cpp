#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <stdio.h>
#include <stdlib.h> // abs
#include <math.h> // sqrt
#include <cmath> // pow, M_PI

// global objects/variables declaration
ros::Publisher publisher;
ros::Subscriber subscriber;
turtlesim::Pose pose_initial;
turtlesim::Pose pose_current;

// global functions declaration
double degreeToRadian(double degree);
double radianToDegree (double radian);
void poseCallBack (const turtlesim::Pose::ConstPtr &message);
void move(double speed, double distance, bool isForward);
void rotate(double speed_degree, double distance_degree, bool clockwise);

int main (int argc, char **argv) {
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle nodeHandle;

    publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    subscriber = nodeHandle.subscribe("turtle1/pose", 1, poseCallBack);

    move(1.0, 1.0, true);
    rotate(30, 90, true);

    return 0;
}

double degreeToRadian(double degree) {
    return degree * M_PI / 180;
}

double radianToDegree(double radian) {
    return radian / M_PI * 180;
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
        ros::spinOnce();
        pose_initial = pose_current;
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

void rotate(double speed_degree, double distance_degree, bool clockwise) {
    geometry_msgs::Twist velocity;
    ros::Rate rate(200);

    // decide velocity 
    if (clockwise) {
        velocity.angular.z = -degreeToRadian(abs(speed_degree));
    }
    else {
        velocity.angular.z = degreeToRadian(abs(speed_degree));
    }
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    // store initial pose
     do {
        ros::spinOnce();
        pose_initial = pose_current;
        rate.sleep();
    } while(pose_initial.x == 0);

    // decide domain flag
    bool positiveFlagPrev, positiveFlag;
    if (pose_current.theta >= 0) {
        positiveFlag = true;
    }
    else {
        positiveFlag = false;
    }

    double degree_current = 0, offset = 0;
    while (degree_current < distance_degree) {
        // decide whether 360 degree offset is needed
        positiveFlagPrev = positiveFlag;
        if (pose_current.theta >= 0) {
            positiveFlag = true;
        }
        else {
            positiveFlag = false;
        }

        if (clockwise && !positiveFlagPrev && positiveFlag) {
            offset -= degreeToRadian(360);
        }
        else if (!clockwise && positiveFlagPrev && !positiveFlag) {
            offset += degreeToRadian(360);
        }

        // calculate distance
        degree_current = radianToDegree(abs(pose_current.theta - pose_initial.theta + offset));

        // log 
        ROS_INFO("Degree: %lf", degree_current);

        // publish velocity (move)
        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();
    } 

     // check degree
    degree_current = radianToDegree(abs(pose_current.theta - pose_initial.theta));
        
    // log 
    ROS_INFO("Degree: %lf", degree_current);
    ROS_INFO("Reached");

    // publish velocity (stop)
    velocity.angular.z = 0;
    publisher.publish(velocity);
    ros::spinOnce();
}



