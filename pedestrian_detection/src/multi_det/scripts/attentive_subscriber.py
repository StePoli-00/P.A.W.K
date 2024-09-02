#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import mediapipe as mp

class PoseSubscriber:
    def __init__(self):
        rospy.init_node('attentive_subscriber', anonymous=True)
        rospy.loginfo("node started")
        self.subscriber = rospy.Subscriber('/robot/front_rgbd_camera/rgb/image_raw', Image, self.callback)
        self.bridge = CvBridge()
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.holistic.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        img_h, img_w, img_c = image.shape
        face_2d = []
        face_3d = []
        attentive = "Not attentive"

        if results.face_landmarks:
            for idx, lm in enumerate(results.face_landmarks.landmark):
                if idx in [33, 263, 1, 61, 291, 199]:
                    if idx == 1:
                        nose_2d = (lm.x * img_w, lm.y * img_h)
                        nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)
                    x, y = int(lm.x * img_w), int(lm.y * img_h)
                    face_2d.append([x, y])
                    face_3d.append([x, y, lm.z])

            face_2d = np.array(face_2d, dtype=np.float64)
            face_3d = np.array(face_3d, dtype=np.float64)
            focal_length = 1 * img_w
            cam_matrix = np.array([[focal_length, 0, img_w / 2],
                                   [0, focal_length, img_h / 2],
                                   [0, 0, 1]])
            distortion_matrix = np.zeros((4, 1), dtype=np.float64)

            success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)
            rmat, jac = cv2.Rodrigues(rotation_vec)
            angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
            x = angles[0] * 360
            y = angles[1] * 360
            z = angles[2] * 360

            if y < -15:
                face_text = "Looking Left"
            elif y > 15:
                face_text = "Looking Right"
            elif x < -15:
                face_text = "Looking Down"
            elif x > 15:
                face_text = "Looking Up"
            else:
                face_text = "Looking Forward"

            if results.pose_landmarks:
                torso_vector = self.calculate_torso_vector(results.pose_landmarks)
                if self.is_facing_camera(torso_vector):
                    label = "Body facing camera"
                    if face_text == "Looking Forward":
                        attentive = "Attentive"
                else:
                    label = "Body not facing camera"

                cv2.putText(image, label, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)
            p1 = (int(nose_2d[0]), int(nose_2d[1]))
            p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))

            cv2.line(image, p1, p2, (255, 0, 0), 3)
            cv2.putText(image, face_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(image, "x: " + str(np.round(x, 2)), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(image, "y: " + str(np.round(y, 2)), (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(image, "z: " + str(np.round(z, 2)), (500, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        cv2.putText(image, attentive, (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
        if results.face_landmarks:
            self.mp_drawing.draw_landmarks(image=image,
                                           landmark_list=results.face_landmarks,
                                           connections=self.mp_holistic.FACEMESH_CONTOURS)

        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image=image, landmark_list=results.pose_landmarks, connections=self.mp_holistic.POSE_CONNECTIONS)

        cv2.imshow('Head and Body Attention Detection', image)
        cv2.waitKey(1)

    def calculate_torso_vector(self, landmarks):
        try:
            left_shoulder = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_SHOULDER].x, landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_SHOULDER].y])
            right_shoulder = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER].x, landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER].y])
            left_hip = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_HIP].x, landmarks.landmark[self.mp_holistic.PoseLandmark.LEFT_HIP].y])
            right_hip = np.array([landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_HIP].x, landmarks.landmark[self.mp_holistic.PoseLandmark.RIGHT_HIP].y])
            shoulder_vector = right_shoulder - left_shoulder
            hip_vector = right_hip - left_hip
            torso_vector = (shoulder_vector + hip_vector) / 2
            return torso_vector
        except IndexError:
            return None

    def is_facing_camera(self, torso_vector, threshold=0.9):
        if torso_vector is None:
            return False
        normal_orientation = np.array([-1, 0])
        dot_product = np.dot(torso_vector, normal_orientation)
        cos_theta = dot_product / (np.linalg.norm(torso_vector) * np.linalg.norm(normal_orientation))
        return cos_theta > threshold

if __name__ == '__main__':
    try:
        PoseSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
