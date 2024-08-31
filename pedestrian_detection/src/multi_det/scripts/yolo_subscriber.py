#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    cv2.imshow("Pose Detection", cv_image)
    cv2.waitKey(1)

def pose_detection_listener():
    rospy.init_node('pose_detection_subscriber', anonymous=True)
    rospy.Subscriber("pose_detected_image", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    pose_detection_listener()
