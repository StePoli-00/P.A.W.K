#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, PointStamped
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy.optimize import minimize

class MoveAndAvoid:
    def __init__(self):
        rospy.init_node('mpc_algorithm', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/robot/robotnik_base_control/cmd_vel', Twist, queue_size=10)

        self.odom_sub = rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.odom_callback)

        self.static_obstacle_sub = rospy.Subscriber('/static_obstacle_distance', PointStamped, self.static_obstacle_callback)
        rospy.loginfo("Subscribed to /static_obstacle_distance")

        self.dynamic_obstacle_sub = rospy.Subscriber('/dynamic_obstacle_distance', PointStamped, self.dynamic_obstacle_callback)
        rospy.loginfo("Subscribed to /dynamic_obstacle_distance")

        self.target_position = (6.0, 0.0)  

        self.current_position = None

        self.distance_to_static_obstacle = float('inf')
        self.distance_to_dynamic_obstacle = float('inf')

        self.avoiding = False

        self.twist = Twist()

        self.omega1 = 1.0  
        self.omega2 = 0.6  
        self.omega3 = 0.8  

        self.previous_error_angle = 0.0

    def odom_callback(self, msg):
        try:
            self.current_position = msg.pose.pose.position
        except Exception as e:
            rospy.logerr(f"Exception in odom_callback: {e}")

    def static_obstacle_callback(self, msg):
        try:
            self.distance_to_static_obstacle = msg.point.x
            rospy.loginfo(f"Distance to static obstacle: {self.distance_to_static_obstacle}")
        except Exception as e:
            rospy.logerr(f"Exception in static_obstacle_callback: {e}")

    def dynamic_obstacle_callback(self, msg):
        try:
            self.distance_to_dynamic_obstacle = msg.point.x
            rospy.loginfo(f"Distance to dynamic obstacle: {self.distance_to_dynamic_obstacle}")
        except Exception as e:
            rospy.logerr(f"Exception in dynamic_obstacle_callback: {e}")

    def calculate_distance(self, pos1, pos2):
        try:
            return math.sqrt((pos2.x - pos1.x)**2 + (pos2.y - pos1.y)**2)
        except Exception as e:
            rospy.logerr(f"Exception in calculate_distance: {e}")
            return 0.0

    def calculate_cost_function(self, u):
        try:
            if self.current_position:
                target_point = Point()  
                target_point.x = self.target_position[0]
                target_point.y = self.target_position[1]
                distance_to_goal = self.calculate_distance(self.current_position, target_point)
            else:
                distance_to_goal = float('inf')  
            
            phi_goal = distance_to_goal**2

            epsilon = 1e-6
            
            phi_obs_static = 1.0 / (self.distance_to_static_obstacle - u[0])**2 if self.distance_to_static_obstacle - u[0] > epsilon else float('inf')
            phi_obs_dynamic = 1.0 / (self.distance_to_dynamic_obstacle - u[0])**2 if self.distance_to_dynamic_obstacle - u[0] > epsilon else float('inf')

            speed_penalty = (1.0 / u[0]) if u[0] > epsilon else float('inf')

            J = self.omega1 * phi_goal + self.omega2 * phi_obs_static + self.omega3 * phi_obs_dynamic + speed_penalty
            return J
        except Exception as e:
            rospy.logerr(f"Exception in calculate_cost_function: {e}")
            return float('inf')

    def optimize_control(self, avoiding=False):
        try:
            def cost_function(u):
                return self.calculate_cost_function(u)

            if avoiding:
                bounds = [(0.1, 0.5), (-1.0, 1.0)]  
            else:
                def cost_function_linear_only(u):
                    return self.calculate_cost_function([u[0], 0.0])
                
                bounds = [(0.1, 0.5), (0.0, 0.0)]  

            initial_guess = np.array([0.2, 0.0])  
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
        rate = rospy.Rate(20)  
        target_point = Point()
        target_point.x = self.target_position[0]
        target_point.y = self.target_position[1]

        while not rospy.is_shutdown():
            try:
                if self.current_position:
                    rospy.loginfo(f"Current position: ({self.current_position.x}, {self.current_position.y})")
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
                        if self.distance_to_dynamic_obstacle < 1.5:  
                            rospy.loginfo("Very close obstacle detected, taking evasive action!")

                            self.avoiding = True

                            u_optimal = self.optimize_control(avoiding=True)

                            self.twist.linear.x = u_optimal[0]
                            self.twist.angular.z = u_optimal[1]

                        else:
                            if self.avoiding:
                                rospy.loginfo("Obstacle avoided. Returning to target")

                                self.avoiding = False

                            u_optimal = self.optimize_control(avoiding=False)

                            angle_to_target = math.atan2(target_point.y - self.current_position.y,
                                                        target_point.x - self.current_position.x)

                            current_yaw = math.atan2(self.current_position.y, self.current_position.x)
                            print(f"CURRENT YAW:{current_yaw}")
                            print(f"ANGLE TO TARGET: {angle_to_target}")
                            
                            error_angle = angle_to_target - current_yaw
                            error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

                            kp = 0.5  
                            kd = 0.1  

                            delta_error_angle = error_angle - self.previous_error_angle
                            self.previous_error_angle = error_angle

                            dead_zone = 0.15

                            self.twist.linear.x = u_optimal[0]

                            if abs(error_angle) < dead_zone:
                                self.twist.angular.z = 0.0  
                            else:
                                self.twist.angular.z = kp * error_angle + kd * delta_error_angle

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