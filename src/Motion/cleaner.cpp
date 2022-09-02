#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <stdio.h>
#include <stdlib.h> // abs
#include <math.h>   // sqrt
#include <cmath>    // pow, M_PI

// global objects/variables declaration
ros::Publisher publisher;
ros::Subscriber subscriber;
turtlesim::Pose pose_initial;
turtlesim::Pose pose_current;

// global functions declaration
double degreeToRadian(double degree);
double radianToDegree(double radian);
void poseCallBack(const turtlesim::Pose::ConstPtr &message);
void move(double speed, double distance, bool isForward);
void rotate(double speed_degree, double distance_degree, bool clockwise);
void goToGoal1(turtlesim::Pose goal, double distance_tol);
void goToGoal2(turtlesim::Pose goal, double tolerance);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle nodeHandle;

    publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    subscriber = nodeHandle.subscribe("turtle1/pose", 1, poseCallBack);

    turtlesim::Pose goalPose;
    goalPose.x = 9;
    goalPose.y = 9;
    goToGoal1(goalPose, 0.1);

    goalPose.x = 1;
    goalPose.y = 1;
    goToGoal2(goalPose, 0.1);

    return 0;
}

double degreeToRadian(double degree)
{
    return degree * M_PI / 180;
}

double radianToDegree(double radian)
{
    return radian / M_PI * 180;
}

void poseCallBack(const turtlesim::Pose::ConstPtr &message)
{
    pose_current.x = message->x;
    pose_current.y = message->y;
    pose_current.theta = message->theta;
}

void move(double speed, double distance, bool isForward = true)
{
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);

    // decide velocity
    if (isForward)
    {
        velocity.linear.x = abs(speed);
    }
    else
    {
        velocity.linear.x = -abs(speed);
    }
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;

    // store initial pose
    int i = 0;
    do
    {
        ros::spinOnce();
        pose_initial = pose_current;
        rate.sleep();
        i++;
    } while (pose_initial.x == 0 && i < 30);

    double distance_current = 0;
    do
    {
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

void rotate(double speed_degree, double distance_degree, bool clockwise = true)
{
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);

    // decide velocity
    if (clockwise)
    {
        velocity.angular.z = -degreeToRadian(abs(speed_degree));
    }
    else
    {
        velocity.angular.z = degreeToRadian(abs(speed_degree));
    }
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;

    // store initial pose
    int i = 0;
    do
    {
        ros::spinOnce();
        pose_initial = pose_current;
        rate.sleep();
        i++;
    } while (pose_initial.x == 0 && i < 30);

    // decide domain flag
    bool positiveFlagPrev, positiveFlag;
    if (pose_current.theta >= 0)
    {
        positiveFlag = true;
    }
    else
    {
        positiveFlag = false;
    }

    double degree_current = 0, offset = 0;
    while (degree_current < distance_degree)
    {
        // decide whether 360 degree offset is needed
        positiveFlagPrev = positiveFlag;
        if (pose_current.theta >= 0)
        {
            positiveFlag = true;
        }
        else
        {
            positiveFlag = false;
        }

        if (clockwise && !positiveFlagPrev && positiveFlag)
        {
            offset -= degreeToRadian(360);
        }
        else if (!clockwise && positiveFlagPrev && !positiveFlag)
        {
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

void goToGoal1(turtlesim::Pose goal, double distance_tol)
{
    double distance_linear, signed_distance_angular;
    double offset = 0, Kl = 0.4, Ka = 1.6;
    double angular_tol = distance_tol/10;
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);

    // get current pose
    int i = 0;
    do
    {
        ros::spinOnce();
        rate.sleep();
        i++;
    } while (pose_current.x == 0 && i < 30);

    // angular
    do
    {
        // calculate angle
        goal.theta = atan((goal.y - pose_current.y) / (goal.x - pose_current.x)); // goal relative to turtle (origin)
        if ((goal.x - pose_current.x) < 0 &&
            (goal.y - pose_current.y) > 0) // quadrant II
        {
            offset = degreeToRadian(180);
        }
        else if ((goal.x - pose_current.x) < 0 &&
                 (goal.y - pose_current.y) < 0) // quadrant III
        {
            offset = -degreeToRadian(180);
        }
        else
        {
            offset = 0;
        }
        signed_distance_angular = goal.theta - pose_current.theta + offset;

        // publish velocity message
        velocity.angular.z = Ka * signed_distance_angular;

        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();

        // log info
        ROS_INFO("Angle left: %lf\n", signed_distance_angular);

    } while (abs(signed_distance_angular) > angular_tol);

    velocity.angular.z = 0;

    do
    {
        // calculate distance
        distance_linear = abs(sqrt(pow((pose_current.x - goal.x), 2) + pow((pose_current.y - goal.y), 2)));

        // publish velocity message
        velocity.linear.x = Kl * distance_linear;
        
        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();

        // log info
        ROS_INFO("Distance left: %lf\n", distance_linear);

    } while (distance_linear > distance_tol);

    // publish velocity (stop)
    velocity.linear.x = 0;
    publisher.publish(velocity);
    ros::spinOnce();
}

void goToGoal2(turtlesim::Pose goal, double tolerance)
{
    double distance_linear, signed_distance_angular;
    double offset = 0, Kl = 0.4, Ka = 1.6;
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);

    // get current pose
    int i = 0;
    do
    {
        ros::spinOnce();
        rate.sleep();
        i++;
    } while (pose_current.x == 0 && i < 30);

    do
    {
        // calculate distance
        distance_linear = abs(sqrt(pow((pose_current.x - goal.x), 2) + pow((pose_current.y - goal.y), 2)));

        // calculate angle
        goal.theta = atan((goal.y - pose_current.y) / (goal.x - pose_current.x)); // goal relative to turtle (origin)
        if ((goal.x - pose_current.x) < 0 &&
            (goal.y - pose_current.y) > 0) // quadrant II
        {
            offset = degreeToRadian(180);
        }
        else if ((goal.x - pose_current.x) < 0 &&
                 (goal.y - pose_current.y) < 0) // quadrant III
        {
            offset = -degreeToRadian(180);
        }
        else
        {
            offset = 0;
        }
        signed_distance_angular = goal.theta - pose_current.theta + offset;

        // publish velocity message
        velocity.linear.x = Kl * distance_linear;
        velocity.angular.z = Ka * signed_distance_angular;

        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();

        // log info
        ROS_INFO("");
        printf("    Distance left: %lf\n", distance_linear);
        printf("    Angle left: %lf\n", signed_distance_angular);

    } while (distance_linear > tolerance);

    // publish velocity (stop)
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    publisher.publish(velocity);
    ros::spinOnce();
}
