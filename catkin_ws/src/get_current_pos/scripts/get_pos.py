#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def amcl_pose_callback(data):
    # Estrai la posizione e l'orientamento
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    orientation = data.pose.pose.orientation
    rospy.loginfo("Posizione: x=%f, y=%f, orientamento=%s", x, y, orientation)

def listener():
    # Inizializza il nodo ROS
    rospy.init_node('get_current_pos', anonymous=True)
    
    # Sottoscriviti al topic /amcl_pose
    rospy.Subscriber("robot/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

    # Mantieni il nodo in esecuzione
    rospy.spin()

if __name__ == '__main__':
    listener()
