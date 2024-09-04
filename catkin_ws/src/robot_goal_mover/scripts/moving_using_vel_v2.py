#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import tf.transformations as tf_transformations

class MoveAndStop:
    def __init__(self):
        rospy.init_node('move_and_stop', anonymous=True)

        # Imposta il publisher per il comando di movimento
        self.cmd_vel_pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)

        # Sottoscrivi ai dati di odometria
        self.odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

        # Imposta la posizione di destinazione (in metri)
        self.target_position = Point(5.0, 5.0, 0.0)  # esempio: x = 5 metri, y = 5 metri

        # Variabili per la gestione della posizione
        self.current_position = None
        self.current_orientation = None

        # Parametri di controllo
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 1.0
        self.linear_tolerance = 0.5  # Distanza alla quale consideriamo raggiunta la posizione
        self.angular_tolerance = 0.1  # Tolleranza angolare

    def odom_callback(self, msg):
        # Aggiorna la posizione e l'orientamento corrente del robot
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def calculate_distance(self, pos1, pos2):
        # Calcola la distanza euclidea tra due punti (x, y)
        return math.sqrt((pos2.x - pos1.x)**2 + (pos2.y - pos1.y)**2)

    def calculate_heading(self, pos1, pos2):
        # Calcola l'angolo tra la posizione corrente e la destinazione
        return math.atan2(pos2.y - pos1.y, pos2.x - pos1.x)

    def get_yaw_from_quaternion(self, quaternion):
        # Converti l'orientamento del robot (quaternion) in angolo yaw
        _, _, yaw = tf_transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return yaw

    def run(self):
        rate = rospy.Rate(20) 
        
        while not rospy.is_shutdown():
            if self.current_position and self.current_orientation:
                # Calcola la distanza alla destinazione
                distance_to_goal = self.calculate_distance(self.current_position, self.target_position)
                
                # Calcola l'angolo di direzione
                desired_heading = self.calculate_heading(self.current_position, self.target_position)

                # Calcola l'angolo corrente (sotto forma di yaw)
                current_yaw = self.get_yaw_from_quaternion(self.current_orientation)

                # Calcola l'errore angolare
                angular_error = desired_heading - current_yaw

                # Normalizza l'errore angolare per [-pi, pi]
                angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))

                # Crea un messaggio Twist per il controllo
                self.twist = Twist()
                
                # Calcola velocità angolare e lineare
                if abs(angular_error) > self.angular_tolerance:
                    self.twist.angular.z = max(min(self.max_angular_velocity, 0.5 * angular_error), -self.max_angular_velocity)
                    self.twist.linear.x = 0.0
                else:
                    self.twist.angular.z = 0.0
                    if distance_to_goal > self.linear_tolerance:
                        self.twist.linear.x = min(self.max_linear_velocity, 0.5 * distance_to_goal)
                    else:
                        self.twist.linear.x = 0.0
                
                if distance_to_goal < self.linear_tolerance and abs(angular_error) < self.angular_tolerance:
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    rospy.loginfo("Target position reached. Stopping robot.")
                    break

                self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MoveAndStop()
        node.run()
    except rospy.ROSInterruptException:
        pass