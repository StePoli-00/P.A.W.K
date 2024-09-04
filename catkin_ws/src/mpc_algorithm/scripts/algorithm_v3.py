#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, PointStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy.optimize import minimize
from tf.transformations import euler_from_quaternion

class MoveAndAvoid:
    def __init__(self):
        rospy.init_node('mpc_algorithm', anonymous=True)

        # Publisher per il comando di movimento
        self.cmd_vel_pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)

        # Subscriber per odometria
        self.odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

        # Subscriber per la distanza dagli ostacoli statici
        self.static_obstacle_sub = rospy.Subscriber('/static_obstacle_distance', PointStamped, self.static_obstacle_callback)
        rospy.loginfo("Subscribed to /static_obstacle_distance")

        # Subscriber per la distanza dagli ostacoli dinamici
        self.dynamic_obstacle_sub = rospy.Subscriber('/dynamic_obstacle_distance', PointStamped, self.dynamic_obstacle_callback)
        rospy.loginfo("Subscribed to /dynamic_obstacle_distance")

        # Imposta la coordinata di destinazione (in metri)
        self.target_position = (6.0, 0.0)  

        # Variabili per la gestione della distanza e posizione
        self.current_position = None
        self.current_yaw = 0.0

        # Variabili per le distanze dagli ostacoli
        self.distance_to_static_obstacle = float('inf')
        self.distance_to_dynamic_obstacle = float('inf')

        # Variabili di stato
        self.avoiding = False
        self.initial_yaw = None  # Per memorizzare l'orientamento iniziale durante l'evitamento

        # Crea un messaggio Twist
        self.twist = Twist()

        # Pesi della funzione di costo
        self.omega1 = 1.0  # Maggiore peso per il goal per una maggiore sensibilità
        self.omega2 = 0.6  # Maggiore peso per gli ostacoli statici
        self.omega3 = 0.8  # Maggiore peso per gli ostacoli dinamici

        self.previous_error_angle = 0.0

        # Soglia per evitare ostacoli dinamici
        self.dynamic_obstacle_avoidance_threshold = 1.1  # Modifica questa soglia in base alle tue necessità

    def quaternion_to_yaw(self, quaternion):
        """
        Convert a quaternion to yaw (z-axis rotation).
        """
        try:
            orientation_q = quaternion
            (roll, pitch, yaw) = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])
            return yaw
        except Exception as e:
            rospy.logerr(f"Exception in quaternion_to_yaw: {e}")
            return 0.0

    def odom_callback(self, msg):
        try:
            # Aggiorna la posizione corrente del robot
            self.current_position = msg.pose.pose.position

            # Estrai il quaternion dall'odometria
            orientation_q = msg.pose.pose.orientation

            # Calcola il yaw dal quaternion
            self.current_yaw = self.quaternion_to_yaw(orientation_q)
            rospy.loginfo(f"Current yaw: {self.current_yaw}")

        except Exception as e:
            rospy.logerr(f"Exception in odom_callback: {e}")

    def static_obstacle_callback(self, msg):
        try:
            # Aggiorna la distanza dal più vicino ostacolo statico
            self.distance_to_static_obstacle = msg.point.x
            rospy.loginfo(f"Distance to static obstacle: {self.distance_to_static_obstacle}")
        except Exception as e:
            rospy.logerr(f"Exception in static_obstacle_callback: {e}")

    def dynamic_obstacle_callback(self, msg):
        try:
            # Aggiorna la distanza dal più vicino ostacolo dinamico
            self.distance_to_dynamic_obstacle = msg.point.x
            rospy.loginfo(f"Distance to dynamic obstacle: {self.distance_to_dynamic_obstacle}")
        except Exception as e:
            rospy.logerr(f"Exception in dynamic_obstacle_callback: {e}")

    def calculate_distance(self, pos1, pos2):
        try:
            # Calcola la distanza euclidea tra due punti (x, y)
            return math.sqrt((pos2.x - pos1.x)**2 + (pos2.y - pos1.y)**2)
        except Exception as e:
            rospy.logerr(f"Exception in calculate_distance: {e}")
            return 0.0

    def calculate_cost_function(self, u):
        try:
            # Calcola la distanza attuale dal punto finale
            if self.current_position:
                target_point = Point()  # Crea un oggetto Point per il target
                target_point.x = self.target_position[0]
                target_point.y = self.target_position[1]
                distance_to_goal = self.calculate_distance(self.current_position, target_point)
            else:
                distance_to_goal = float('inf')  # Se la posizione non è disponibile, consideriamo un costo infinito
            
            phi_goal = distance_to_goal**2

            # Evita divisioni per zero con una piccola costante
            epsilon = 1e-6
            
            phi_obs_static = 1.0 / (self.distance_to_static_obstacle - u[0])**2 if self.distance_to_static_obstacle - u[0] > epsilon else float('inf')
            phi_obs_dynamic = 1.0 / (self.distance_to_dynamic_obstacle - u[0])**2 if self.distance_to_dynamic_obstacle - u[0] > epsilon else float('inf')

            # Penalizza la lentezza quando non ci sono ostacoli vicini
            speed_penalty = (1.0 / u[0]) if u[0] > epsilon else float('inf')

            # Funzione di costo totale
            J = self.omega1 * phi_goal + self.omega2 * phi_obs_static + self.omega3 * phi_obs_dynamic + speed_penalty
            return J
        except Exception as e:
            rospy.logerr(f"Exception in calculate_cost_function: {e}")
            return float('inf')

    def optimize_control(self, avoiding=False):
        try:
            # Definisci la funzione di costo per l'ottimizzazione
            def cost_function(u):
                return self.calculate_cost_function(u)

            if avoiding:
                # Ottimizza sia la velocità lineare che angolare durante l'evitamento
                bounds = [(0.2, 0.8), (-1.0, 1.0)]  # Velocità lineare tra 0.2 e 0.8, velocità angolare tra -1 e 1
            else:
                # Ottimizza solo la velocità lineare quando non ci sono ostacoli
                def cost_function_linear_only(u):
                    return self.calculate_cost_function([u[0], 0.0])
                
                bounds = [(0.1, 0.5), (0.0, 0.0)]  # Solo velocità lineare

            initial_guess = np.array([0.2, 0.0])  # Velocità lineare e angolare iniziali più basse
            result = minimize(cost_function, initial_guess, bounds=bounds)
            
            if result.success:
                return result.x
            else:
                rospy.logwarn("Optimization failed")
                return initial_guess
        except Exception as e:
            rospy.logerr(f"Exception in optimize_control: {e}")
            return np.array([0.0, 0.0])

    def run(self):
        rate = rospy.Rate(25)  # Maggiore frequenza di aggiornamento

        # Crea un oggetto Point per la posizione del target
        target_point = Point()
        target_point.x = self.target_position[0]
        target_point.y = self.target_position[1]

        while not rospy.is_shutdown():
            try:
                if self.current_position:
                    rospy.loginfo(f"Current position: ({self.current_position.x}, {self.current_position.y})")
                    rospy.loginfo(f"Current yaw: {self.current_yaw}")

                    distance_to_goal = self.calculate_distance(self.current_position, target_point)
                    rospy.loginfo(f"Distance to goal: {distance_to_goal}")

                    if distance_to_goal < 0.2:
                        # Fermati
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                        rospy.loginfo("Target reached. Stopping robot.")
                        break
                    else:
                        if self.distance_to_dynamic_obstacle < self.dynamic_obstacle_avoidance_threshold:
                            rospy.loginfo("Very close dynamic obstacle detected, taking evasive action!")

                            # Imposta lo stato di evitamento e memorizza l'orientamento iniziale
                            if not self.avoiding:
                                self.avoiding = True
                                self.initial_yaw = self.current_yaw

                            # Determina la direzione per evitare l'oggetto
                            # Modifica la velocità angolare in base alla posizione dell'ostacolo
                            if self.current_yaw >= 0:
                                self.twist.angular.z = 1.0  # Sterza a sinistra se il robot è orientato verso destra
                            else:
                                self.twist.angular.z = -1.0  # Sterza a destra se il robot è orientato verso sinistra

                            # Riduci la velocità lineare durante l'evitamento
                            self.twist.linear.x = 0.3

                        else:
                            if self.avoiding:
                                rospy.loginfo("Obstacle avoided. Returning to target")

                                # Ritorna a uno stato di movimento verso il target
                                self.avoiding = False

                                # Calcola l'angolo verso il target
                                angle_to_target = math.atan2(target_point.y - self.current_position.y,
                                                            target_point.x - self.current_position.x)

                                # Calcola l'errore angolare
                                print(f"CURRENT YAW: {self.current_yaw}")
                                print(f"ANGLE TT: {angle_to_target}")
                                error_angle = angle_to_target - self.current_yaw
                                error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

                                # Guadagno proporzionale e derivativo
                                kp = 0.5
                                kd = 0.1

                                # Calcola la derivata dell'errore angolare
                                delta_error_angle = error_angle - self.previous_error_angle
                                self.previous_error_angle = error_angle

                                # Zona morta per evitare piccole oscillazioni
                                dead_zone = 0.05

                                # Imposta le velocità
                                u_optimal = self.optimize_control(avoiding=False)
                                self.twist.linear.x = u_optimal[0]

                                if abs(error_angle) < dead_zone:
                                    self.twist.angular.z = 0.0  # Nessuna correzione se l'errore è piccolo
                                else:
                                    self.twist.angular.z = kp * error_angle + kd * delta_error_angle
                            else:
                                # Ottimizza solo la velocità lineare
                                u_optimal = self.optimize_control(avoiding=False)
                                self.twist.linear.x = u_optimal[0]
                                self.twist.angular.z = 0.0  # Nessun comando angolare se non ci sono ostacoli

                        # Pubblica il comando di movimento
                        self.cmd_vel_pub.publish(self.twist)
            except Exception as e:
                rospy.logerr(f"Exception in run loop: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        move_and_avoid = MoveAndAvoid()
        move_and_avoid.run()
    except rospy.ROSInterruptException:
        pass