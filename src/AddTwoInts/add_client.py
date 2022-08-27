import sys
import rospy
from beginning_with_ros.srv import AddTwoInts
from beginning_with_ros.srv import AddTwoIntsRequest
from beginning_with_ros.srv import AddTwoIntsResponse

def add_client(x, y):
    # Check if service exist
    rospy.wait_for_service('add_two_ints')
    try:
        # Initialize service provider
        serviceProxy = rospy.ServiceProxy('add_two_ints', AddTwoInts)

        # Get response
        response = serviceProxy(x, y)

        return response.sum

    except rospy.ServiceException(error):
        rospy.loginfo("Service call failed: {}".format(error))

if __name__ == "__main__":
    # Initialize node
    rospy.init_node("add_client_py")

    # Check argument number
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        response_sum = add_client(x, y)

        rospy.loginfo("")
        print("    Request: {}, {}".format(x, y))
        print("    Response from server: {}".format(response_sum))

    else:
        rospy.loginfo("Incorrect number of arguments - Need: x y")
