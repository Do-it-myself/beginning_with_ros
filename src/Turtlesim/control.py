import rospy
from geometry_msgs.msg import Twist

from pynput import keyboard
from pynput.keyboard import Key
import math

## Talker
def talker():
    # Node
    rospy.init_node('turtle_control',anonymous=True)

    # Publisher
    publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(1000)

    # Publishing
    twist = Twist()

    # Keyboard
    def on_press(key):
        if key == Key.esc:
            return False
        elif key == Key.ctrl:
            return False
        elif key == Key.up:
            twist.linear.x = 1
        elif key == Key.down:
            twist.linear.x = -1
        elif key == Key.left:
            twist.angular.z = math.pi/3
        elif key == Key.right:
            twist.angular.z = -math.pi/3
        publisher.publish(twist)
        rate.sleep()
        

    def on_release(key):
        if key == Key.up:
            twist.linear.x = 0
        if key == Key.down:
            twist.linear.x = 0
        if key == Key.left:
            twist.angular.z = 0
        if key == Key.right:
            twist.angular.z = 0
        publisher.publish(twist)
        rate.sleep()

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
