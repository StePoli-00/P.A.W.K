#!/usr/bin/python3

import rospy
import numpy as np
import json
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy
from pathlib import Path
# Global variables to store sensor data
current_pose = None
front_laser_scan = None
rear_laser_scan = None
root_path,_=str(Path(__file__).resolve()).split("/Smart-Robotics-Project")
params_path=f"{root_path}/Smart-Robotics-Project/catkin_ws/src/mpc_algorithm/params/fra_params.json"

def load_params():
    with open(params_path,"r") as f:
        return json.load(f)



params=load_params()
# Parameters for MPC
omega_goal = params["omega_goal"]
omega_obstacle = params["omega_obstacle"]

# Laser scan parameters
angle_min = params["angle_min"]  # -135 degrees
angle_max = params["angle_max"]    # 135 degrees
angle_increment = params["angle_increment"]    # 0.5 degrees

#thresholds
thr_static_obs=params["thr_static_obs"]

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose  # Extract the pose from the message with covariance

def front_laser_scan_callback(msg):
    global front_laser_scan
    front_laser_scan = msg.ranges

def rear_laser_scan_callback(msg):
    global rear_laser_scan
    rear_laser_scan = msg.ranges

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
        if distance_to_static_obs < thr_static_obs:  # Threshold for considering an obstacle close
            phi_obs_static += 1 / (distance_to_static_obs ** 2)

    # Total cost function J(x,u)
    cost = omega_goal * phi_goal + omega_obstacle * phi_obs_static
    return cost

def get_obstacle_positions(front_scan, rear_scan):
    obstacles = []
    num_readings_front = len(front_scan)
    num_readings_rear = len(rear_scan)
    
    # Process front laser scan
    for i in range(num_readings_front):
        r = front_scan[i]
        if r < 2.5:  # Consider obstacles within 2.5 meters
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            obstacles.append([x, y])
    
    # Process rear laser scan
    for i in range(num_readings_rear):
        r = rear_scan[i]
        if r < 5:  # Consider obstacles within 2.5 meters
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            obstacles.append([x, y])

    return obstacles

def mpc_control_loop(goal_pose):
    rate = rospy.Rate(20)  # Control loop rate: 20 Hz
    delta_t = 0.05  # Time step between iterations

    while not rospy.is_shutdown():
        # Ensure sensor data is available
        if current_pose is None or front_laser_scan is None or rear_laser_scan is None:
            rospy.loginfo("Waiting for sensor data...")
            rate.sleep()
            continue

        # Get static obstacle positions from sensor data
        static_obstacles = get_obstacle_positions(front_laser_scan, rear_laser_scan)

        # Calculate control input that minimizes the cost function
        best_cost = float('inf')
        best_twist = Twist()

        # Simulate different control inputs (e.g., velocities) and evaluate the cost
        for linear_vel in np.linspace(params["lb_lin_vel"], params["up_lin_vel"], 5):  # Test different linear velocities
            for angular_vel in np.linspace(params["lb_ang_vel"], params["up_ang_vel"], 10):  # Test different angular velocities

                # Create a copy of the current pose to simulate
                simulated_pose = deepcopy(current_pose)

                # Extract current orientation (as angle theta)
                current_orientation = simulated_pose.orientation
                _, _, current_theta = euler_from_quaternion([
                    current_orientation.x, 
                    current_orientation.y, 
                    current_orientation.z, 
                    current_orientation.w
                ])

                # Simulate the new position using the kinematic model
                simulated_pose.position.x += linear_vel * np.cos(current_theta) * delta_t
                simulated_pose.position.y += linear_vel * np.sin(current_theta) * delta_t

                # Simulate the new orientation
                new_theta = current_theta + angular_vel * delta_t
                simulated_pose.orientation = quaternion_from_euler(0, 0, new_theta)  # Convert theta to quaternion

                # Calculate cost for this control input
                cost = calculate_cost(simulated_pose, goal_pose, static_obstacles)

                # Update the control command if the cost is lower
                if cost < best_cost:
                    best_cost = cost
                    best_twist.linear.x = linear_vel
                    best_twist.angular.z = angular_vel

        # Publish the best control command found
        cmd_vel_pub.publish(best_twist)

        rospy.loginfo(f"Best command: Linear velocity: {best_twist.linear.x}, Angular velocity: {best_twist.angular.z}")
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('mpc_algorithm')

    # Publishers for control commands
    cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

    # Subscribers for sensor data
    rospy.Subscriber('/robot/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/robot/front_laser/scan', LaserScan, front_laser_scan_callback)
    rospy.Subscriber('/robot/rear_laser/scan', LaserScan, rear_laser_scan_callback)

    # Define a goal position for the robot to navigate towards
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = params["goal_pose_x"] # Example goal position (2 meters behind)
    goal_pose.pose.position.y = params["goal_pose_y"] 

    # Start the MPC control loop
    mpc_control_loop(goal_pose)
