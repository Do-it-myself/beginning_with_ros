import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def poseCallBack(message):
    global x_current, y_current, theta_current
    x_current = message.x
    y_current = message.y
    theta_current = message.theta

def move(publisher, speed, distance, isForward):
    # calculate distance
    global x_current, y_current, theta_current, distance_current
    distance_current = 

if __name__ == "__main__":
    rospy.init_node("cleaner_py")
    publisher = rospy.Publisher("turtle1/cmd_vel", Twist, 100)
    subscriber = rospy.Subscriber("turtle1/pose", Pose, poseCallBack)
    move()