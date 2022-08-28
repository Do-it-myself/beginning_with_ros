import rospy
from beginning_with_ros.srv import RectangleAreaService

def area_handler(request):
    response = request.height * request.width
    rospy.loginfo("")
    print('    Request from client: {}, {}'.format(request.height, request.width))
    print('    Response: {}'.format(response))
    return response

def area_server():
    rospy.init_node("area_server_py")
    rospy.Service("rectangle_area", RectangleAreaService, area_handler)
    rospy.loginfo("Server ready")
    rospy.spin()

if __name__ == "__main__":
    area_server()