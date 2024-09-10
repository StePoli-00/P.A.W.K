#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import numpy as np
import cv2
import mediapipe as mp

class PoseSubscriber:
    def __init__(self):
        rospy.init_node('attentive_subscriber', anonymous=True)
        self.subscriber = rospy.Subscriber('/robot/wrist_rgbd/rgb/image_raw', Image, self.callback)
        self.attention_publisher = rospy.Publisher('/attentive_subscriber', Float32, queue_size=10)  
        self.mp_holistic = mp.solutions.holistic
        self.bridge = CvBridge()
        self.holistic = self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=1, circle_radius=1)

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.holistic.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        if results.pose_landmarks:
            torso_vector, shoulder_distance, torso_height = self.calculate_torso_vector(results.pose_landmarks)
            body_facing_camera = self.is_facing_camera(torso_vector, shoulder_distance, torso_height)
            face_facing_camera = self.is_face_facing_camera(results.pose_landmarks, torso_height)

            if body_facing_camera and face_facing_camera:
                label = "Body and Face facing camera"
                attention_level = 1.0
            elif body_facing_camera or face_facing_camera:
                label = "Body facing camera, Face not facing" if body_facing_camera else "Face facing camera, Body not facing"
                attention_level = 0.5
            else:
                label = "Body and Face not facing camera"
                attention_level = 0.1

            # Pubblica il livello di attenzione
            self.attention_publisher.publish(Float32(attention_level))

            # Display the result on the image
            cv2.putText(image, label, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            self.mp_drawing.draw_landmarks(image=image, landmark_list=results.pose_landmarks, connections=self.mp_holistic.POSE_CONNECTIONS)

        # Show the processed image
        cv2.imshow('Head and Body Pose Detection', image)
        cv2.waitKey(1)

    def calculate_torso_vector(self, landmarks):
        try:
            left_shoulder = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_SHOULDER].x, landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_SHOULDER].y])
            right_shoulder = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER].x, landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER].y])
            left_hip = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_HIP].x, landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_HIP].y])
            right_hip = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_HIP].x, landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_HIP].y])

            shoulder_vector = right_shoulder - left_shoulder
            torso_vector = shoulder_vector

            shoulder_distance = np.linalg.norm(shoulder_vector)
            torso_height = np.linalg.norm((left_shoulder + right_shoulder) / 2 - (left_hip + right_hip) / 2)
            
            return torso_vector, shoulder_distance, torso_height
        except IndexError:
            return None, None, None

    def is_facing_camera(self, torso_vector, shoulder_distance, torso_height, threshold=0.9, distance_ratio_threshold=0.2):
        if torso_vector is None:
            return False

        normal_orientation = np.array([-1, 0])  # Normal direction for a body facing the camera
        dot_product = np.dot(torso_vector, normal_orientation)
        cos_theta = dot_product / (np.linalg.norm(torso_vector) * np.linalg.norm(normal_orientation))

        # Check if the person is turned towards the camera
        if cos_theta > threshold:
            distance_ratio_threshold *= (torso_height / 0.10321167712968708)  # Adjust ratio threshold similarly
            a = shoulder_distance / torso_height > distance_ratio_threshold
            if a:
                return True

        return False

    def is_face_facing_camera(self, face_landmarks, torso_height, eye_distance_threshold=0.002):
        if face_landmarks is None:
            return False

        left_eye = np.array([face_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_EYE].x, face_landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_EYE].y])
        right_eye = np.array([face_landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_EYE].x, face_landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_EYE].y])
        
        eye_distance = np.linalg.norm(left_eye - right_eye)
        
        # Adjust eye distance threshold based on torso height
        eye_distance_threshold *= (torso_height / 0.10321167712968708)  # Use the example torso height to scale the threshold
        return eye_distance > eye_distance_threshold


if __name__ == '__main__':
    try:
        PoseSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
