import rospy
from beginning_with_ros.msg import IOTsensor

def iot_callback_function(message):
    rospy.loginfo("")
    print("   Name: {}".format(message.name))
    print("   ID: {}".format(message.id))
    print("   Temperature: {}".format(message.temperature))
    print("   Humidity: {}".format(message.humidity))
    

def iot_listener():
    rospy.init_node('iot_listener')
    rospy.Subscriber('iot_info', IOTsensor, iot_callback_function)
    rospy.spin()

if __name__ == "__main__":
    try:
        iot_listener()
    except rospy.ROSInterruptException:
        pass



