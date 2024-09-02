#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import numpy as np

class ObstacleDistanceCalculator:
    def __init__(self):
        rospy.init_node('obstacle_distance_calculator')

        # Publisher per la distanza dal più vicino ostacolo statico
        self.static_obstacle_pub = rospy.Publisher('/static_obstacle_distance', PointStamped, queue_size=10)
        
        # Publisher per la distanza dal più vicino ostacolo dinamico (per esempio)
        self.dynamic_obstacle_pub = rospy.Publisher('/dynamic_obstacle_distance', PointStamped, queue_size=10)
        
        # Subscriber per il LIDAR (ostacoli statici)
        rospy.Subscriber('/robot/front_laser/scan', LaserScan, self.lidar_callback)
        
        # Placeholder per gli ostacoli dinamici
        rospy.Subscriber('/robot/rear_laser/scan', LaserScan, self.dynamic_lidar_callback)
        
        self.rate = rospy.Rate(10)
    
    def lidar_callback(self, msg):
        # Estrai i dati del LIDAR
        ranges = np.array(msg.ranges)
        
        # Rimuovi valori infiniti o NaN
        ranges = ranges[np.isfinite(ranges)]
        
        if len(ranges) > 0:
            min_distance = np.min(ranges)
        else:
            min_distance = float('inf')
        
        # Pubblica la distanza minima trovata dagli ostacoli statici
        static_obstacle_msg = PointStamped()
        static_obstacle_msg.header = Header(stamp=rospy.Time.now(), frame_id=msg.header.frame_id)
        static_obstacle_msg.point.x = min_distance
        static_obstacle_msg.point.y = 0.0
        static_obstacle_msg.point.z = 0.0
        
        self.static_obstacle_pub.publish(static_obstacle_msg)
        rospy.loginfo(f"Distanza dal più vicino ostacolo dinamico: {min_distance} m")
    
    def dynamic_lidar_callback(self, msg):
        # Estrai i dati del LIDAR per ostacoli dinamici
        ranges = np.array(msg.ranges)
        
        # Rimuovi valori infiniti o NaN
        ranges = ranges[np.isfinite(ranges)]
        
        if len(ranges) > 0:
            min_distance = np.min(ranges)
        else:
            min_distance = float('inf')
        
        # Pubblica la distanza minima trovata dagli ostacoli dinamici
        dynamic_obstacle_msg = PointStamped()
        dynamic_obstacle_msg.header = Header(stamp=rospy.Time.now(), frame_id=msg.header.frame_id)
        dynamic_obstacle_msg.point.x = min_distance
        dynamic_obstacle_msg.point.y = 0.0
        dynamic_obstacle_msg.point.z = 0.0
        
        self.dynamic_obstacle_pub.publish(dynamic_obstacle_msg)
        rospy.loginfo(f"Distanza dal più vicino ostacolo statico: {min_distance} m")
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = ObstacleDistanceCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass