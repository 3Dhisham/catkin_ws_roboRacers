import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

image_publisher = rospy.Publisher("threshold", Image, queue_size=10)
image_subscriber = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, process_image, queue_size=10)
ros_img_bridge = CvBridge()
cv_img = []
intrinsic_params = np.array([[383.7944641113281, 0.0, 322.3056945800781],
                             [0.0, 383.7944641113281, 241.67051696777344],
                             [0.0, 0.0, 1.0]])

distortion_coeff = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
rw_coord = np.array([[0.5, 0.2, 0.0],
                     [0.5, -0.2, 0.0],
                     [0.8, 0.2, 0.0],
                     [0.8, -0.2, 0.0],
                     [1.1, 0.2, 0.0],
                     [1.1, -0.2, 0.0]])
# init node
rospy.init_node("camera_calibration")
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    rate.sleep()


def process_image(msg):
    cv_img = ros_img_bridge.imgmsg_to_cv2(msg, "mono8")

    max_val = 255
    threshold = 210
    cv_img = cv2.rectangle(cv_img, (0, 0), (640, 110), 0, cv2.FILLED)
    cv_img = cv2.rectangle(cv_img, (0, 240), (640, 480), 0, cv2.FILLED)
    ret, cv_img = cv2.threshold(cv_img, threshold, max_val, cv2.THRESH_BINARY)

    #   Python: cv2.rectangle(img, pt1, pt2, color, thickness, lineType, shift)
    # bottom left
    cv2.rectangle(cv_img, (175, 205), (235, 265), 150, 2)
    # bottom right
    cv2.rectangle(cv_img, (465, 205), (525, 255), 150, 2)
    # mid left
    cv2.rectangle(cv_img, (215, 135), (265, 175), 150, 2)
    # mid right
    cv2.rectangle(cv_img, (415, 125), (455, 165), 150, 2)
    # top left
    cv2.rectangle(cv_img, (265, 100), (285, 125), 150, 2)
    # top right
    cv2.rectangle(cv_img, (405, 100), (425, 125), 150, 2)

    image_publisher.publish(ros_img_bridge.cv2_to_imgmsg(cv_img, "mono8"))


if __name__ == "__main__":
    process_image(msg)
