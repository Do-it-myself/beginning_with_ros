import rospy
from std_msgs.msg import String

def talker():
    ## ----- NODE ----- ##
    # Create and initialize node
    # arg: node_name, anonymous (create different ID for different nodes)
    rospy.init_node('talker_py', anonymous=True)

    ## ----- PUBLISHER ----- ##
    # Create publisher object 
    # arg: topic_name, topic_type, queue_size
    publisher = rospy.Publisher('test_talk', String, queue_size=10)

    # Define publication rate (Hz)
    rate = rospy.Rate(1) 

    # Create message
    i = 0
    while not rospy.is_shutdown():
        msg_str = "Counter: %s" % i
        # Put string to log
        rospy.loginfo("Python - %s" % msg_str)
        # Publish message
        publisher.publish(msg_str)
        # Control message sending rate
        rate.sleep()
        
        i+=1

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

