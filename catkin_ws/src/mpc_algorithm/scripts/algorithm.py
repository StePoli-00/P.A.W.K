#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
import math
import numpy as np

class MoveAndAvoid:
    def __init__(self):
        rospy.init_node('mpc_algorithm', anonymous=True)

        # Publisher per il comando di movimento
        self.cmd_vel_pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)

        # Subscriber per odometria
        self.odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

        # Subscriber per la distanza dagli ostacoli statici
        rospy.Subscriber('/static_obstacle_distance', PointStamped, self.static_obstacle_callback)
        
        # Subscriber per la distanza dagli ostacoli dinamici
        rospy.Subscriber('/dynamic_obstacle_distance', PointStamped, self.dynamic_obstacle_callback)

        # Imposta la distanza desiderata (in metri)
        self.target_distance = 6.0  # esempio: 6 metri

        # Variabili per la gestione della distanza e posizione
        self.start_position = None
        self.current_position = None
        self.distance_traveled = 0.0

        # Variabili per le distanze dagli ostacoli
        self.distance_to_static_obstacle = float('inf')
        self.distance_to_dynamic_obstacle = float('inf')

        # Crea un messaggio Twist
        self.twist = Twist()
        self.twist.linear.x = 0.5
        self.twist.angular.z = 0.0

        # Pesi della funzione di costo
        self.omega1 = 1.0  # Peso per il goal
        self.omega2 = 1.0  # Peso per gli ostacoli statici
        self.omega3 = 1.0  # Peso per gli ostacoli dinamici

    def odom_callback(self, msg):
        # Aggiorna la posizione corrente del robot
        self.current_position = msg.pose.pose.position

        if self.start_position is None:
            self.start_position = self.current_position
        else:
            # Calcola la distanza percorsa
            self.distance_traveled = self.calculate_distance(self.start_position, self.current_position)

    def static_obstacle_callback(self, msg):
        # Aggiorna la distanza dal più vicino ostacolo statico
        self.distance_to_static_obstacle = msg.point.x

    def dynamic_obstacle_callback(self, msg):
        # Aggiorna la distanza dal più vicino ostacolo dinamico
        self.distance_to_dynamic_obstacle = msg.point.x

    def calculate_distance(self, start, end):
        # Calcola la distanza tra due punti (x, y)
        return math.sqrt((end.x - start.x)**2 + (end.y - start.y)**2)

    def calculate_cost_function(self, u):
        # Calcolo dei termini della funzione di costo
        distance_to_goal = self.target_distance - self.distance_traveled
        phi_goal = (distance_to_goal - u[0])**2

        # Evita divisioni per zero con una piccola costante
        epsilon = 1e-6
        
        phi_obs_static = 1.0 / (self.distance_to_static_obstacle - u[0])**2 if self.distance_to_static_obstacle > epsilon else float('inf')
        phi_obs_dynamic = 1.0 / (self.distance_to_dynamic_obstacle - u[0])**2 if self.distance_to_dynamic_obstacle > epsilon else float('inf')

        # Funzione di costo totale
        J = self.omega1 * phi_goal + self.omega2 * phi_obs_static + self.omega3 * phi_obs_dynamic
        return J

    def optimize_control(self):
        # Possibili velocità lineari per il robot
        u_options = [np.array([0.1]), np.array([0.2]), np.array([0.3]), np.array([0.4]), np.array([0.5])]
        costs = [self.calculate_cost_function(u) for u in u_options]
        u_optimal = u_options[np.argmin(costs)]
        return u_optimal

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
                # Ottimizza il controllo
                u_optimal = self.optimize_control()
                self.twist.linear.x = u_optimal[0]
                self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = MoveAndAvoid()
        node.run()
    except rospy.ROSInterruptException:
        pass