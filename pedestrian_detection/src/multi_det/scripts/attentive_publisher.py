#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PosePublisher:
    def __init__(self):
        rospy.init_node('attentive_publisher', anonymous=True)
        self.publisher = rospy.Publisher('attention_image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.rate = rospy.Rate(10)  # 10 Hz

    def process_and_publish(self):
        while not rospy.is_shutdown():
            success, image = self.cap.read()
            if not success:
                rospy.logerr("Failed to capture image")
                continue

            image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.publisher.publish(image_message)

            self.rate.sleep()

    def __del__(self):
        self.cap.release()

if __name__ == '__main__':
    try:
        node = PosePublisher()
        node.process_and_publish()
    except rospy.ROSInterruptException:
        pass
