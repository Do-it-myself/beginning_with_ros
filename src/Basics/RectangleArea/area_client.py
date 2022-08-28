import sys
import rospy
from beginning_with_ros.srv import RectangleAreaService

def area_client(x, y):
    rospy.wait_for_service("rectangle_area")
    try:
        serviceProxy = rospy.ServiceProxy("rectangle_area", RectangleAreaService)
        response = serviceProxy(x, y)
        return response.area
    except rospy.ServiceException():
        rospy.loginfo("Service call failed")

if __name__ == "__main__":
    rospy.init_node("area_client_py")
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
        area = area_client(x, y)
        rospy.loginfo("")
        print("    Request: {}, {}".format(x, y))
        print("    Response from server: {}".format(area))
    else:
        rospy.loginfo("Incorrect number of arguments - Need: x y")
