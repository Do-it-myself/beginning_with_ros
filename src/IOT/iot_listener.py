import rospy
from beginning_with_ros.msg import IOTsensor

def iot_callback_function(message):
    rospy.loginfo("")
    

def iot_listener():
    rospy.init_node('iot_listener', anonymous=True)
    rospy.Subscriber('iot_info', IOTsensor, iot_callback_function)
    rospy.spin()

if __name__ == "__main__":
    try:
        iot_listener()
    except rospy.ROSInterruptException:
        pass



