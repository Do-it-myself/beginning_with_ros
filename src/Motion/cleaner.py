import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

pose_current = Pose()
pose_initial = Pose()

def poseCallBack(message):
    global pose_current
    pose_current.x = message.x
    pose_current.y = message.y
    pose_current.theta = message.theta

def move(publisher, speed, distance, isForward):
    rate = rospy.Rate(100)

    # decide velocity
    velocity = Twist()
    if isForward:
        velocity.linear.x = abs(speed)
    else:
        velocity.linear.x = -abs(speed)
    velocity.linear.y, velocity.linear.z = 0, 0
    velocity.angular.x, velocity.angular.y, velocity.angular.z, = 0, 0, 0
    
    # store initial pose
    global pose_initial, pose_current, distance_current
    pose_initial.x = pose_current.x
    pose_initial.y = pose_current.y
    pose_initial.theta = pose_current.theta
    i = 0
    while (pose_initial.x == 0 and i < 10):
        pose_initial.x = pose_current.x
        pose_initial.y = pose_current.y
        pose_initial.theta = pose_current.theta
        rate.sleep()
        i += 1

    distance_current = 0
    while (distance_current < distance):
        # calculate distance
        distance_current = math.sqrt((pose_current.x - pose_initial.x)**2 + (pose_current.y - pose_initial.y)**2)

        # log info
        rospy.loginfo("Distance: {}".format(distance_current))

        # publish velocity message (move)
        publisher.publish(velocity)
        rate.sleep()

    rospy.loginfo("Distance: {}".format(distance_current))
    rospy.loginfo("Reached")

    # publish velocity message (stop)
    velocity.linear.x = 0
    publisher.publish(velocity)

def rotate(publisher, speed_degree, distance_degree, clockwise):
    rate = rospy.Rate(100)

    # decide velocity
    velocity = Twist()
    if clockwise:
        velocity.angular.z = -math.radians(abs(speed_degree))
    else:
        velocity.angular.z = math.radians(abs(speed_degree))
    velocity.linear.x, velocity.linear.y, velocity.linear.z = 0, 0, 0
    velocity.angular.x, velocity.angular.y = 0, 0
    
    # store initial pose
    global pose_initial, pose_current, distance_current
    pose_initial.x = pose_current.x
    pose_initial.y = pose_current.y
    pose_initial.theta = pose_current.theta
    i = 0
    while (pose_initial.x == 0 and i < 10):
        pose_initial.x = pose_current.x
        pose_initial.y = pose_current.y
        pose_initial.theta = pose_current.theta
        rate.sleep()
        i += 1

    # decide domain flag
    if pose_current.theta >= 0:
        positiveFlag = True
    else:
        positiveFlag = False

    degree_current = 0
    offset = 0
    while (degree_current < distance_degree):
        # decide whether 360 degree offset is needed
        positiveFlagPrev = positiveFlag
        if pose_current.theta >= 0:
            positiveFlag = True
        else:
            positiveFlag = False
        
        if clockwise and not positiveFlagPrev and positiveFlag:
            offset -= math.radians(360)
        elif not clockwise and positiveFlagPrev and not positiveFlag:
            offset += math.radians(360)
        
        # calculate distance
        degree_current = math.degrees(abs(pose_current.theta - pose_initial.theta + offset))

        # log info
        rospy.loginfo("Degree: {}".format(degree_current))

        # publish velocity message (move)
        publisher.publish(velocity)
        rate.sleep()

    rospy.loginfo("Distance: {}".format(degree_current))
    rospy.loginfo("Reached")

    # publish velocity message (stop)
    velocity.angular.z = 0
    publisher.publish(velocity)

def goToGoal1(publisher, goal, distance_tol):
    Kl = 0.4
    Ka = 1.6
    angular_tol = distance_tol/10
    velocity = Twist()
    rate = rospy.Rate(100)

    # get current pose
    i = 0
    while (pose_current.x == 0 and i < 10):
        rate.sleep()
        i += 1

    # angular
    signed_distance_angular = angular_tol + 1 # to make initial condition true
    while (abs(signed_distance_angular) > angular_tol):
        goal.theta = math.atan((goal.y - pose_current.y) / (goal.x - pose_current.x))
        if ((goal.x - pose_current.x) < 0 and (goal.y - pose_current.y) > 0): # quadrant II
            offset = math.radians(180)
        elif ((goal.x - pose_current.x) < 0 and (goal.y - pose_current.y) < 0): # quadrant III
            offset = -math.radians(180)
        else:
            offset = 0
        signed_distance_angular = goal.theta - pose_current.theta + offset

        # publish velocity message
        velocity.angular.z = Ka * signed_distance_angular
        publisher.publish(velocity)
        rate.sleep()

        # log info
        rospy.loginfo("Angle left: {}".format(signed_distance_angular))

    velocity.angular.z = 0

    # linear
    distance_linear = distance_tol + 1
    while (distance_linear > distance_tol):
        # calculate distance 
        distance_linear = abs(math.sqrt((pose_current.x - goal.x)**2 + (pose_current.y - goal.y)**2))
        
        # publish velocity message
        velocity.linear.x = Kl * distance_linear
        publisher.publish(velocity)
        rate.sleep()

        # log info
        rospy.loginfo("Distance left: {}".format(distance_linear))

    # publish velocity (stop)
    velocity.linear.x = 0
    publisher.publish(velocity)

def goToGoal2(publisher, goal, tolerance):
    Kl = 0.4
    Ka = 1.6
    velocity = Twist()
    rate = rospy.Rate(100)

    # get current pose
    i = 0
    while (pose_current.x == 0 and i < 10):
        rate.sleep()
        i += 1

    distance_linear = tolerance + 1 # to make initial condition true
    while (distance_linear > tolerance):
        # calculate distance 
        distance_linear = abs(math.sqrt((pose_current.x - goal.x)**2 + (pose_current.y - goal.y)**2))

        # calculate angle
        goal.theta = math.atan((goal.y - pose_current.y) / (goal.x - pose_current.x))
        if ((goal.x - pose_current.x) < 0 and (goal.y - pose_current.y) > 0): # quadrant II
            offset = math.radians(180)
        elif ((goal.x - pose_current.x) < 0 and (goal.y - pose_current.y) < 0): # quadrant III
            offset = -math.radians(180)
        else:
            offset = 0
        signed_distance_angular = goal.theta - pose_current.theta + offset

        # publish velocity message
        velocity.linear.x = Kl * distance_linear
        velocity.angular.z = Ka * signed_distance_angular
        publisher.publish(velocity)
        rate.sleep()

        # log info
        rospy.loginfo("")
        print("    Distance left: {}".format(distance_linear))
        print("    Angle left: {}".format(signed_distance_angular))
    
    # publish velocity (stop)
    velocity.linear.x = 0
    velocity.angular.z = 0
    publisher.publish(velocity)

if __name__ == "__main__":
    rospy.init_node("cleaner_py")
    publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=100)
    subscriber = rospy.Subscriber("turtle1/pose", Pose, poseCallBack)
    move(publisher, 1, 1, True)
    rotate(publisher, 30, 90, False)

    goalPose = Pose()
    goalPose.x = 9
    goalPose.y = 9
    goToGoal2(publisher, goalPose, 0.1)

    goalPose.x = 1
    goalPose.y = 8
    goToGoal1(publisher, goalPose, 0.1)