#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Errore nel convertire l'immagine: {e}")
        return

    cv2.imshow("Immagine dal Robot", cv_image)
    cv2.waitKey(1)


def listener():
    rospy.init_node('image_listener', anonymous=True)

    rospy.Subscriber("<TOPIC>", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()