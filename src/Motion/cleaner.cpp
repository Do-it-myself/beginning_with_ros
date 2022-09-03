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
void setDesiredOrientation(double speed_degree, double desired_angle_degree);
void spiralClean1(double linear_factor, double angular_factor);
void spiralClean2(double linear_factor, double angular_factor);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cleaner");
    ros::NodeHandle nodeHandle;

    publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
    subscriber = nodeHandle.subscribe("turtle1/pose", 1, poseCallBack);

    turtlesim::Pose goalPose;
    spiralClean1(5, 50);
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
    double angular_tol = distance_tol / 10;
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

void setDesiredOrientation(double speed_degree, double desired_angle_degree)
{
    bool clockwise;
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);

    // store initial pose
    int i = 0;
    do
    {
        ros::spinOnce();
        rate.sleep();
        i++;
    } while (pose_current.x == 0 && i < 30);

    // check magnitude
    double distance_degree = abs(desired_angle_degree - radianToDegree(pose_current.theta));
    if (distance_degree > 180)
    { // obtuse
        distance_degree = 360 - distance_degree;
        // check clockwise
        clockwise = ((desired_angle_degree - radianToDegree(pose_current.theta)) < 0) ? false : true;
    }
    else
    { // acute
        // check clockwise
        clockwise = ((desired_angle_degree - radianToDegree(pose_current.theta)) < 0) ? true : false;
    }

    // rotate
    rotate(speed_degree, distance_degree, clockwise);
}

void spiralClean1(double linear, double angular)
{
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);
    double angle_total, offset;
    bool positiveFlagPrev, positiveFlag;
    
    // store initial pose
    int i = 0;
    do
    {
        ros::spinOnce();
        pose_initial = pose_current;
        rate.sleep();
        i++;
    } while (pose_current.x == 0 && i < 30);

    positiveFlag = (pose_current.theta >= 0) ? true : false;

    while (pose_current.x > 0.5 &&
           pose_current.y > 0.5 &&
           pose_current.x < 10.5 &&
           pose_current.y < 10.5)
    {
        // calculate total angle moved
        positiveFlagPrev = positiveFlag;
        positiveFlag = (pose_current.theta >= 0) ? true : false;
        if (positiveFlagPrev && !positiveFlag)
        {
            offset += degreeToRadian(360);
        }
        angle_total = abs(pose_current.theta - pose_initial.theta + offset);

        // set 
        velocity.linear.x = linear;
        velocity.angular.z = angular / sqrt(angle_total * angle_total + 1);

        // publish velocity (move)
        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();     
    }

    // publish velocity (stop)
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    publisher.publish(velocity);
    ros::spinOnce();   
}

void spiralClean2(double linear, double angular)
{
    geometry_msgs::Twist velocity;
    ros::Rate rate(100);
    
    // store initial pose
    int i = 0;
    do
    {
        ros::spinOnce();
        rate.sleep();
        i++;
    } while (pose_current.x == 0 && i < 30);

    while (pose_current.x > 0.5 &&
           pose_current.y > 0.5 &&
           pose_current.x < 10.5 &&
           pose_current.y < 10.5)
    {
        // set 
        velocity.linear.x = linear;
        velocity.angular.z = angular;
        linear = linear + 0.1;

        // publish velocity (move)
        publisher.publish(velocity);
        ros::spinOnce();
        rate.sleep();     
    }

    // publish velocity (stop)
    velocity.linear.x = 0;
    velocity.angular.z = 0;
    publisher.publish(velocity);
    ros::spinOnce();   
}