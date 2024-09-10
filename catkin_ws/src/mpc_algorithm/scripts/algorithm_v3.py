#!/usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from copy import deepcopy

current_pose = None
front_laser_scan = None
rear_laser_scan = None

omega_goal = 1.0
omega_obstacle = 0.8
beta = 1.0  

angle_min = -2.3561999797821045  
angle_max = 2.3561999797821045   
angle_increment = 0.008726666681468487  

def beta_callback(msg):
    global beta
    beta = msg.data  

def amcl_pose_callback(msg):
    global current_pose
    current_pose = msg.pose.pose 

def front_laser_scan_callback(msg):
    global front_laser_scan
    front_laser_scan = msg.ranges

def rear_laser_scan_callback(msg):
    global rear_laser_scan
    rear_laser_scan = msg.ranges

def calculate_cost(current_pose, goal_pose, obstacles_static):
    distance_to_goal = np.linalg.norm(np.array([current_pose.position.x, current_pose.position.y]) -
                                      np.array([goal_pose.pose.position.x, goal_pose.pose.position.y]))

    phi_goal = distance_to_goal ** 2

    phi_obs_static = 0
    min_distance_to_obstacle = float('inf')
    for obs in obstacles_static:
        distance_to_static_obs = np.linalg.norm(np.array([current_pose.position.x, current_pose.position.y]) - obs)
        min_distance_to_obstacle = min(min_distance_to_obstacle, distance_to_static_obs)
        if distance_to_static_obs < 2:  
            phi_obs_static += 1 / (distance_to_static_obs ** 2)

    global omega_obstacle
    omega_obstacle = max(0.3, (1 / ((min_distance_to_obstacle + 0.05) * (1 + (1 - beta)))))
    rospy.loginfo(f"Omega Obstacle: {omega_obstacle}")
    rospy.loginfo(f"Beta: {beta}")

    # J(x,u)
    cost = omega_goal * phi_goal + omega_obstacle * phi_obs_static
    return cost

def calculate_target_angle(current_pose, goal_pose):
    goal_x = goal_pose.pose.position.x
    goal_y = goal_pose.pose.position.y

    current_x = current_pose.position.x
    current_y = current_pose.position.y

    dx = goal_x - current_x
    dy = goal_y - current_y

    target_angle = np.arctan2(dy, dx)

    current_orientation = current_pose.orientation
    _, _, current_theta = euler_from_quaternion([
        current_orientation.x, 
        current_orientation.y, 
        current_orientation.z, 
        current_orientation.w
    ])

    angle_diff = current_theta - target_angle
    angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

    return angle_diff

def get_obstacle_positions(front_scan, rear_scan):
    obstacles = []
    num_readings_front = len(front_scan)
    num_readings_rear = len(rear_scan)
    
    for i in range(num_readings_front):
        r = front_scan[i]
        if r < 1.5:  
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            obstacles.append([x, y])
    
    for i in range(num_readings_rear):
        r = rear_scan[i]
        if r < 1.5:  
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            obstacles.append([x, y])

    return obstacles

def mpc_control_loop(goal_pose):
    rate = rospy.Rate(30)
    delta_t = 0.033 

    while not rospy.is_shutdown():
        if current_pose is None or front_laser_scan is None or rear_laser_scan is None:
            rospy.loginfo("Waiting for sensor data...")
            rate.sleep()
            continue

        static_obstacles = get_obstacle_positions(front_laser_scan, rear_laser_scan)
        min_distance_to_obstacle = min([np.linalg.norm(np.array([current_pose.position.x, current_pose.position.y]) - np.array(obs)) for obs in static_obstacles] + [float('inf')])

        distance_to_goal = np.linalg.norm(np.array([current_pose.position.x, current_pose.position.y]) -
                                          np.array([goal_pose.pose.position.x, goal_pose.pose.position.y]))

        if distance_to_goal < 0.5:
            rospy.loginfo("Goal reached! Stopping the robot.")
            cmd_vel_pub.publish(Twist())  
            rospy.signal_shutdown("Reached the goal") 
            return

        best_cost = float('inf')
        best_twist = Twist()

        for linear_vel in np.linspace(-1, 0.5, 10):  
            for angular_vel in np.linspace(-0.5, 0.5, 15): 
                
                simulated_pose = deepcopy(current_pose)

                current_orientation = simulated_pose.orientation
                _, _, current_theta = euler_from_quaternion([
                    current_orientation.x, 
                    current_orientation.y, 
                    current_orientation.z, 
                    current_orientation.w
                ])

                simulated_pose.position.x += linear_vel * np.cos(current_theta) * delta_t
                simulated_pose.position.y += linear_vel * np.sin(current_theta) * delta_t

                new_theta = current_theta + angular_vel * delta_t
                if abs(new_theta - current_theta) > abs(angular_vel * delta_t):
                    new_theta = current_theta + np.sign(angular_vel) * abs(angular_vel * delta_t)


                simulated_pose.orientation = quaternion_from_euler(0, 0, new_theta) 

                cost = calculate_cost(simulated_pose, goal_pose, static_obstacles)

                if cost < best_cost:
                    best_cost = cost
                    best_twist.linear.x = linear_vel
                    best_twist.angular.z = angular_vel * omega_obstacle

        if min_distance_to_obstacle > 1.5: 
            target_angle_diff = calculate_target_angle(current_pose, goal_pose)
            best_twist.angular.z = np.clip(target_angle_diff, -0.4, 0.4)  

        cmd_vel_pub.publish(best_twist)

        rospy.loginfo(f"Distance to goal: {distance_to_goal:.2f}")
        rospy.loginfo(f"Best command: Linear velocity: {best_twist.linear.x}, Angular velocity: {best_twist.angular.z}")
        
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('mpc_algorithm')

    cmd_vel_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

    rospy.Subscriber('/robot/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/robot/front_laser/scan', LaserScan, front_laser_scan_callback)
    rospy.Subscriber('/robot/rear_laser/scan', LaserScan, rear_laser_scan_callback)

    rospy.Subscriber('/attentive_subscriber', Float32, beta_callback)

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = -3.0  
    goal_pose.pose.position.y = 0.0

    mpc_control_loop(goal_pose)
