#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def function(message):
    bridge=CvBridge()
    rospy.loginfo("received a video massage/frame")
    converted_frame_to_cv=bridge.imgmsg_to_cv2(message)
    cv2.imshow("camera",converted_frame_to_cv)
    cv2.waitKey(1)


rospy.init_node("image_subscriber",anonymous=True)
rospy.Subscriber("video_topic",Image,callback=function)
rospy.spin()
cv2.destroyAllWindows()