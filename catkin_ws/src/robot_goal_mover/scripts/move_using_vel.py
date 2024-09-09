#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MoveAndStop:
    def __init__(self):
        rospy.init_node('move_and_stop', anonymous=True)

        # Imposta il publisher per il comando di movimento
        self.cmd_vel_pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)

        # Sottoscrivi ai dati di odometria
        self.odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

        # Imposta la distanza desiderata (in metri)
        self.target_distance = 15.0  # esempio: 5 metri

        # Variabili per la gestione della distanza
        self.start_position = None
        self.current_position = None
        self.distance_traveled = 0.0

        # Crea un messaggio Twist
        self.twist = Twist()
        self.twist.linear.x = -0.5
        self.twist.angular.z = 0.0

    def odom_callback(self, msg):
        # Aggiorna la posizione corrente del robot
        self.current_position = msg.pose.pose.position

        if self.start_position is None:
            self.start_position = self.current_position
        else:
            # Calcola la distanza percorsa
            self.distance_traveled = self.calculate_distance(self.start_position, self.current_position)

    def calculate_distance(self, start, end):
        # Calcola la distanza tra due punti (x, y)
        return math.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if self.distance_traveled >= self.target_distance:
                # Fermati
                self.twist.linear.x = 0.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.loginfo("Target distance reached. Stopping robot.")
                break
            else:
                # Continua a muoversi
                self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MoveAndStop()
        node.run()
    except rospy.ROSInterruptException:
        pass