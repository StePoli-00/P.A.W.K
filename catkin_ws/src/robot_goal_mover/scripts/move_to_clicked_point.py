#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from tf import TransformListener

class MoveToClickedPoint:
    def __init__(self):
        rospy.init_node('move_to_clicked_point', anonymous=True)

        # Creare un listener per le trasformazioni TF
        self.tf_listener = TransformListener()

        # Sottoscrivi al topic /clicked_point per ottenere il punto cliccato
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        print(PointStamped)
        print(self.clicked_point_callback)
        # Publisher per inviare il goal al topic /robot/move_base_simple/goal
        self.goal_pub = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=10)

    def clicked_point_callback(self, msg):
        try:
            rospy.loginfo(f"Frame ID ricevuto: {msg.header.frame_id}")
            rospy.loginfo(f"MSG: {msg}")
            
            # Definisci i frame di riferimento
            target_frame = "robot_odom"  # Frame di destinazione
            source_frame = msg.header.frame_id  # Frame di origine del punto cliccato
            
            # Attendi la trasformazione necessaria tra il frame di origine e il frame di destinazione
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))

            # Trasforma il punto cliccato nel frame del robot
            transformed_point = self.tf_listener.transformPoint(target_frame, msg)

            # Creazione del messaggio PoseStamped per inviare il goal
            goal_msg = PoseStamped()

            # Imposta il frame di riferimento
            goal_msg.header.frame_id = target_frame
            goal_msg.header.stamp = rospy.Time.now()

            # Imposta la posizione (x, y, z) basata sul punto cliccato trasformato
            goal_msg.pose.position = transformed_point.point

            # Imposta l'orientamento (qui lasciamo che il robot non ruoti, ma puoi modificare se necessario)
            goal_msg.pose.orientation.w = 1.0

            # Pubblica il goal
            self.goal_pub.publish(goal_msg)
            rospy.loginfo(f"Goal pubblicato: {goal_msg.pose.position.x}, {goal_msg.pose.position.y}")

        except Exception as e:
            rospy.logerr(f"Errore nella trasformazione del punto: {str(e)}")

if __name__ == '__main__':
    try:
        move_to_clicked_point = MoveToClickedPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass