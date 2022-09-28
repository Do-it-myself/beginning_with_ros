import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def image_callback(ros_image):
    global bridge

    # Photo conversion
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    except CvBridgeError as e:
        print("Error:", e)

    # OpenCV
    cv_image_flip = cv2.flip(cv_image, 1)
    (rows, columns, channels) = cv_image_flip.shape
    threshold_image = cv_image_flip
    font = cv2.FONT_HERSHEY_PLAIN
    cv2.putText(threshold_image, "CV Bridge", (175, 50), font, 3, (255, 255, 255), 2)
    cv2.imshow("Image window", threshold_image)
    cv2.waitKey(3)

def main():
    rospy.init_node('image_converter')
    image_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt...")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()