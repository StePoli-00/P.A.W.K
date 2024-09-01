#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
from tf import TransformListener

class PublishAndTransformPoint:
    def __init__(self):
        rospy.init_node('publish_and_transform_point', anonymous=True)

        # Creare un listener per le trasformazioni TF
        self.tf_listener = TransformListener()

        # Publisher per il punto fisso
        self.point_pub = rospy.Publisher('point_topic', PointStamped, queue_size=10)
        
        # Publisher per inviare il goal al topic /robot/move_base_simple/goal
        self.goal_pub = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Definisce la frequenza di pubblicazione
        self.rate = rospy.Rate(10)  # 10 Hz

        # Imposta i valori del punto
        self.fixed_point = PointStamped()
        self.fixed_point.header.frame_id = "robot_map"
        self.fixed_point.point.x = 3.4247219562530518
        self.fixed_point.point.y = 0.056365013122558594
        self.fixed_point.point.z = 0.00562286376953125

    def run(self):
        while not rospy.is_shutdown():
            # Imposta il timestamp e pubblica il punto fisso
            self.fixed_point.header.stamp = rospy.Time.now()
            self.point_pub.publish(self.fixed_point)
            
            # Trasformazione e pubblicazione del goal
            self.publish_goal_from_point()

            # Attende prima di pubblicare il prossimo messaggio
            self.rate.sleep()

    def publish_goal_from_point(self):
        try:
            # Definisci i frame di riferimento
            target_frame = "robot_odom"  # Frame di destinazione
            source_frame = self.fixed_point.header.frame_id  # Frame di origine del punto fisso
            
            # Attendi la trasformazione necessaria tra il frame di origine e il frame di destinazione
            self.tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))

            # Trasforma il punto fisso nel frame del robot
            transformed_point = self.tf_listener.transformPoint(target_frame, self.fixed_point)

            # Creazione del messaggio PoseStamped per inviare il goal
            goal_msg = PoseStamped()

            # Imposta il frame di riferimento
            goal_msg.header.frame_id = target_frame
            goal_msg.header.stamp = rospy.Time.now()

            # Imposta la posizione (x, y, z) basata sul punto fisso trasformato
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
        node = PublishAndTransformPoint()
        node.run()
    except rospy.ROSInterruptException:
        pass