import rospy
from std_msgs.msg import String

def callback_function(message):
    rospy.loginfo("")
    print("   Name: {}".format(message.name))
    print("   ID: {}".format(message.id))
    print("   Temperature: {}".format(message.temperature))
    print("   Humidity: {}".format(message.humidity))

def listener():
    ## ----- NODE ----- ##
    # Create and initialize node
    # arg: node_name, anonymous (create different ID for different nodes)
    rospy.init_node('listener_py', anonymous=True)

    ## ----- SUBSCRIBER ----- ##
    # Create subscriber object
    # arg: topic_name, topic_type, callback_function
    rospy.Subscriber('test_talk', String, callback_function) 

    # Enter listening mode
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
