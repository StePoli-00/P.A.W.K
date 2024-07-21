#!/usr/bin/env
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String 

def image_publisher():
    
    #image_pub=rospy.Publisher("image_topic",Image,queue_size=10)
    image_pub=rospy.Publisher("image_topic",String,queue_size=10)
    rospy.init_node("image_publisher", anonymous=True)
    bridge=CvBridge()
    rate=rospy.Rate(1)

    # img_path="/home/stefano/Desktop/Smart-Robotics-Project/driver_assistance/src/mediapipe/person.jpeg"
    # cv2_image=cv2.imread(img_path)

    while not rospy.is_shutdown():
         rospy.loginfo("Hi")
         image_pub.publish("Hi")
         rate.sleep()
    # while not rospy.is_shutdown():
       
    #     try:
    #         if cv2_image is not None:
    #             ros_image=bridge.cv2_to_imgmsg(cv2_image,"rgb8") #forse rgb8
    #             image_pub.publish(ros_image)
    #             rospy.loginfo("Published Image")
    #         else: 
    #             rospy.loginfo("No Image found ")
    #     except Exception as e:
    #          rospy.logerr(e)
       
    #     rate.sleep()

if __name__=="__main__":
    try:
        image_publisher()
    except rospy.ROSInterruptException :
            pass
    except Exception as e:
         rospy.logerr(e)