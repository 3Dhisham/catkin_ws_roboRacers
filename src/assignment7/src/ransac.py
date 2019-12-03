import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


img_bridge = CvBridge()
sub_img = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, process_img, queue_size=10)
pub_img = rospy.Publisher("detect", Image, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    rate.sleep()

lanes = []
num_samples = 0
threshold = 0

def evaluate(line, threshold):
    m = line
    b = line
    inlier = lanes[np.where((np.abs(b + m * lanes[:, 1] - lanes[:, 0]) / (np.sqrt(1 + (m * m)))) < threshold)]
    outlier = lanes[np.where((np.abs(b + m * lanes[:, 1] - lanes[:, 0]) / (np.sqrt(1 + (m * m)))) >= threshold)]
    return inlier, outlier, inlier.size

def fit_model(data):
    m = (data[0, 0] - data[0, 1] - data[1, 1])
    b = data[1, 0] - m * data[1, 1]
    return m, b

def fit(lanes, num_samples, threshold):
    top_outlier = []
    top_inlier = []
    top_line = (0, 0)
    top_mark = 0    
    for i in range(num_samples):
        random = np.random.choice(lanes.shape[0], 2)
        data = lanes[random]
        line = fit_model(data)
        outlier = evaluate(line, threshold)
        inlier = evaluate(line, threshold)
        mark = evaluate(line, threshold)
        if mark > top_mark:
            top_line = line
            top_mark = mark
            top_outlier = outlier
            top_inlier = inlier
    return top_line, top_inlier, top_outlier

def process_img(img):
    max_val = 255
    threshold = 240
    img = img_bridge.imgmsg_to_cv2(img, "mono8")

    img = cv2.rectangle(img, (0, 0), (620, 100), 0, cv2.FILLED)
    img = cv2.rectangle(img, (0, 250), (620, 450), 0, cv2.FILLED)
    ret, img = cv2.threshold(img, threshold, max_val, cv2.THRESH_BINARY)
    lane = np.argwhere(img==255)

    for i in range(5):
        line = fit(lane, 50, 10)
        outlier = fit(lane, 50, 10)
        inlier = fit(lane, 50, 10)
        m = line
        b = line
        cv2.line(img, (0, b), (640, 640 * m + b), 140, 7)
        lane = outlier
    
    pub_img.publish(img_bridge.cv2_to_imgmsg(img, "mono8"))



if __name__ == "__main__":
    rospy.init_node("detect")
    process_img()
