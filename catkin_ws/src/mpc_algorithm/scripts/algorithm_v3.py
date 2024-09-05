#!/usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

# Global variables to store sensor data
current_pose = None
front_laser_scan = None

# Parameters for MPC
omega_goal = 0.6
omega_3 = 0.7

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose  # Estrarre la posizione dal messaggio con covarianza

# Callback for receiving front laser scan data
def laser_scan_callback(msg):
    global front_laser_scan
    front_laser_scan = msg.ranges

# Calculate the cost function J(x, u) for the MPC
def calculate_cost(current_pose, goal_pose, obstacles_static):
    # Distance from robot to goal
    distance_to_goal = np.linalg.norm(np.array([current_pose.position.x, current_pose.position.y]) -
                                      np.array([goal_pose.pose.position.x, goal_pose.pose.position.y]))

    # Cost for goal (Φ_goal)
    phi_goal = distance_to_goal ** 2

    # Cost for static obstacles (Φ_obs_static)
    phi_obs_static = 0
    for obs in obstacles_static:
        distance_to_static_obs = np.linalg.norm(np.array([current_pose.position.x, current_pose.position.y]) - obs)
        if distance_to_static_obs < 1.0:  # Threshold for considering an obstacle close
            phi_obs_static += 1 / (distance_to_static_obs ** 2)

    # Total cost function J(x,u)
    cost = omega_goal * phi_goal + omega_3 * phi_obs_static
    return cost


# Function to get obstacle positions from laser scan data
def get_obstacle_positions(laser_scan):
    obstacles = []
    angle_increment = 0.5  # Example value, this can be extracted from the LaserScan message
    current_angle = -len(laser_scan) * angle_increment / 2
    for r in laser_scan:
        if r < 10.0:  # Consider obstacles within 10 meters
            x = r * np.cos(current_angle)
            y = r * np.sin(current_angle)
            obstacles.append([x, y])
        current_angle += angle_increment
    return obstacles

# Main MPC optimization loop
def mpc_control_loop(goal_pose):
    rate = rospy.Rate(10)  # Control loop rate: 10 Hz
    while not rospy.is_shutdown():
        # Ensure sensor data is available
        if current_pose is None or front_laser_scan is None:
            rospy.loginfo("Waiting for sensor data...")
            rate.sleep()
            continue

        # Get static obstacle positions from sensor data
        static_obstacles = get_obstacle_positions(front_laser_scan)

        # Calculate control input that minimizes the cost function
        best_cost = float('inf')
        best_twist = Twist()

        # Simulate different control inputs (e.g., velocities) and evaluate the cost
        for linear_vel in np.linspace(-1.3, -0.3, 7):  # Test different linear velocities
            for angular_vel in np.linspace(-1.0, 1.0, 7):  # Test different angular velocities
                simulated_twist = Twist()
                simulated_twist.linear.x = linear_vel
                simulated_twist.angular.z = angular_vel

                # Simulate next state (ignoring dynamics for simplicity)
                #simulated_pose = current_pose  # Assume current_pose gets updated with actual pose
                
                simulated_pose.position.x += simulated_twist.linear.x * np.cos(current_pose.orientation.z) * delta_t
                simulated_pose.position.y += simulated_twist.linear.x * np.sin(current_pose.orientation.z) * delta_t

                # Calculate cost for this control input
                cost = calculate_cost(simulated_pose, goal_pose, static_obstacles)
               
                if cost < best_cost:
                    best_cost = cost
                    best_twist = simulated_twist

        # Publish the best control command
        cmd_vel_pub.publish(best_twist)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('mpc_algorithm')

    # Publishers for control commands
    cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

    # Subscribers for sensor data
    rospy.Subscriber('/robot/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/robot/front_laser/scan', LaserScan, laser_scan_callback)

    # Define a goal position for the robot to navigate towards
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = -2.0  # Example goal position (5 meters ahead)
    goal_pose.pose.position.y = 0.0

    # Start the MPC control loop
    mpc_control_loop(goal_pose)
