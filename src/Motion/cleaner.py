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
    while (pose_initial.x == 0):
        pose_initial.x = pose_current.x
        pose_initial.y = pose_current.y
        pose_initial.theta = pose_current.theta
        rate.sleep()

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

if __name__ == "__main__":
    rospy.init_node("cleaner_py")
    publisher = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=100)
    subscriber = rospy.Subscriber("turtle1/pose", Pose, poseCallBack)
    move(publisher, 1, 1, True)