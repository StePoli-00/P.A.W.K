#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_frame():
    rospy.init_node("camera_publisher", anonymous=True)
    publisher=rospy.Publisher("video_topic",Image,queue_size=60)
    rate=rospy.Rate(1) #3 images per seconds

    capture=cv2.VideoCapture(0)
    if capture.isOpened()==False:
        rospy.logerr("Camera Not found")
    bridge=CvBridge()

    while not rospy.is_shutdown():
        ret,cap=capture.read()
        if ret:
            try:
                image=bridge.cv2_to_imgmsg(cap)
                publisher.publish(image)
                rospy.loginfo("Video frame captured and published")
            except Exception as e:
                rospy.logerr(e)
        
        rate.sleep()

    capture.release()
    
if __name__ == '__main__':
    try:
        publish_frame()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught. Exiting...")
    except Exception as e:
        rospy.logerr("Unhandled exception: {}".format(e))