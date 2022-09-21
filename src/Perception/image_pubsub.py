import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

def image_callback(ros_image):
    global bridge

    # Photo conversion
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print("Error:", e)

    # OpenCV
    (rows, columns, channels) = cv_image.shape
    print("Rows:", rows)
    print("Columns:", columns)
    print("Channels:", channels)
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(cv_image, "CV Bridge", (50, 50), font, 1, (255, 255, 255), 2)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

def main(args):
    rospy.init_node('image_converter')
    image_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt...")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)