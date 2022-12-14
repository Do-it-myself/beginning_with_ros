import rospy
from beginning_with_ros.srv import AddTwoInts

# Provide response
def add_handler(request):
    response = request.a + request.b
    rospy.loginfo("")
    print('    Request from client: {}, {}'.format(request.a, request.b))
    print('    Response: {}'.format(response))
    return response

def add_server():
    rospy.init_node('add_server_py')

    # Initialize service provider
    rospy.Service('add_two_ints', AddTwoInts, add_handler)

    rospy.loginfo('Server ready')

    # Wait for request
    rospy.spin()

if __name__ == "__main__":
    add_server()
