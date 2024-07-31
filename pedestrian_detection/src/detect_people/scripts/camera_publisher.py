#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from pathlib import Path
root_path,_=str(Path(__file__).resolve()).split("/Smart-Robotics-Project")
frames_path=root_path+"/Smart-Robotics-Project/show_frame/frames"

def publish_frame():
    rospy.init_node("camera_publisher_from_kairos", anonymous=True)
    publisher=rospy.Publisher("frame_topic",Image,queue_size=60)
    rate=rospy.Rate(10) #3 images per seconds

    bridge=CvBridge()

    while not rospy.is_shutdown():
        frames=os.listdir(frames_path)
        frames=[os.path.join(frames_path,frame) for frame in frames]

        for frame in frames:
            cv_image=cv2.imread(frame)
            if cv_image is not None:
                image_msg=bridge.cv2_to_imgmsg(cv_image,encoding="bgr8")
                publisher.publish(image_msg)
                rospy.loginfo(f"Image Published: {frame}")
        
        rate.sleep()

    
    
if __name__ == '__main__':
    try:
        publish_frame()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught. Exiting...")
    except Exception as e:
        rospy.logerr("Unhandled exception: {}".format(e))