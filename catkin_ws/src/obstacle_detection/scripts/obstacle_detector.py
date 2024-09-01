#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from costmap_converter.msg import ObstacleArrayMsg
from geometry_msgs.msg import PointStamped
import numpy as np

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector')
        
        # Publisher per la distanza dal più vicino ostacolo statico
        self.static_obstacle_pub = rospy.Publisher('/static_obstacle_distance', PointStamped, queue_size=10)
        
        # Publisher per la distanza dal più vicino ostacolo dinamico
        self.dynamic_obstacle_pub = rospy.Publisher('/dynamic_obstacle_distance', PointStamped, queue_size=10)
        
        # Subscriber per la costmap locale (ostacoli statici)
        rospy.Subscriber('/robot/move_base/local_costmap/costmap', OccupancyGrid, self.costmap_callback)
        
        # Subscriber per il topic degli ostacoli dinamici
        rospy.Subscriber('/robot/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, self.obstacle_callback)
        
        self.rate = rospy.Rate(10)
    
    def costmap_callback(self, costmap):
        # Trova gli ostacoli nella costmap
        data = np.array(costmap.data)
        data = data.reshape((costmap.info.height, costmap.info.width))
        
        # Trova l'indice dell'ostacolo più vicino (occupancy value > 0)
        indices = np.argwhere(data > 0)
        if len(indices) > 0:
            # Calcola la distanza dall'ostacolo più vicino
            distances = np.linalg.norm(indices * costmap.info.resolution, axis=1)
            min_distance = np.min(distances)
        else:
            min_distance = float('inf')
        
        # Pubblica la distanza minima trovata dagli ostacoli statici
        static_obstacle_msg = PointStamped()
        static_obstacle_msg.header.stamp = rospy.Time.now()
        static_obstacle_msg.point.x = min_distance
        static_obstacle_msg.point.y = 0.0
        static_obstacle_msg.point.z = 0.0
        
        self.static_obstacle_pub.publish(static_obstacle_msg)
        rospy.loginfo(f"Distanza dal più vicino ostacolo statico: {min_distance} m")
    
    def obstacle_callback(self, msg):
        if len(msg.obstacles) > 0:
            # Trova la distanza dal più vicino ostacolo dinamico
            distances = [np.linalg.norm([obs.center.x, obs.center.y]) for obs in msg.obstacles]
            min_distance = min(distances)
        else:
            min_distance = float('inf')
        
        # Pubblica la distanza minima trovata dagli ostacoli dinamici
        dynamic_obstacle_msg = PointStamped()
        dynamic_obstacle_msg.header.stamp = rospy.Time.now()
        dynamic_obstacle_msg.point.x = min_distance
        dynamic_obstacle_msg.point.y = 0.0
        dynamic_obstacle_msg.point.z = 0.0
        
        self.dynamic_obstacle_pub.publish(dynamic_obstacle_msg)
        rospy.loginfo(f"Distanza dal più vicino ostacolo dinamico: {min_distance} m")
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    detector = ObstacleDetector()
    detector.run()
