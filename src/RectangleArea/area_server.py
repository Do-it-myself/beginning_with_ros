import rospy
from beginning_with_ros.srv import RectangleAreaService
from beginning_with_ros.srv import RectangleAreaServiceRequest
from beginning_with_ros.srv import  RectangleAreaServiceResponse

def area_handler(request):
    response = request.height * request.width
    