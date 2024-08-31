import numpy as np
import cv2
import mediapipe as mp
import time
import torch

# Initialize YOLOv5 model from Torch Hub
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

# Initialize holistic model
mp_holistic = mp.solutions.holistic
holistic = mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Drawing utilities
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=1, circle_radius=1)

def calculate_torso_vector(landmarks):
    try:
        left_shoulder = np.array([landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER].x, landmarks.landmark[mp_holistic.PoseLandmark.LEFT_SHOULDER].y])
        right_shoulder = np.array([landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER].x, landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_SHOULDER].y])
        left_hip= np.array([landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].x, landmarks.landmark[mp_holistic.PoseLandmark.LEFT_HIP].y])
        right_hip = np.array([landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].x, landmarks.landmark[mp_holistic.PoseLandmark.RIGHT_HIP].y])
        
        shoulder_vector = right_shoulder - left_shoulder
        hip_vector = right_hip - left_hip
        torso_vector = (shoulder_vector + hip_vector) / 2
        return torso_vector
    except IndexError:
        return None

def is_facing_camera(torso_vector, threshold=0.9):
    if torso_vector is None:
        return False
    normal_orientation = np.array([-1, 0])  
    dot_product = np.dot(torso_vector, normal_orientation)
    cos_theta = dot_product / (np.linalg.norm(torso_vector) * np.linalg.norm(normal_orientation))
    return cos_theta > threshold

# Open video capture
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success:
        break

    start = time.time()
    img_h, img_w, img_c = image.shape
    
    # Use YOLOv5 to detect people
    results_yolo = model(image)

    # Process each detected person
    for result in results_yolo.xyxy[0]:  # Loop over each detected person
        if result[5] == 0:  # Class '0' is for person in COCO dataset
            x1, y1, x2, y2 = map(int, result[:4])  # Bounding box coordinates
            cropped_image = image[y1:y2, x1:x2]  # Crop the image to the bounding box

            # Convert cropped image to RGB
            cropped_image_rgb = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
            cropped_image_rgb.flags.writeable = False
            
            # Process the cropped image
            results = holistic.process(cropped_image_rgb)
            
            cropped_image_rgb.flags.writeable = True
            cropped_image_bgr = cv2.cvtColor(cropped_image_rgb, cv2.COLOR_RGB2BGR)
            
            face_2d = []
            face_3d = []

            attentive = "Not attentive"

            if results.face_landmarks:
                for idx, lm in enumerate(results.face_landmarks.landmark):
                    if idx in [33, 263, 1, 61, 291, 199]:
                        if idx == 1:
                            nose_2d = (lm.x * (x2-x1), lm.y * (y2-y1))
                            nose_3d = (lm.x * (x2-x1), lm.y * (y2-y1), lm.z * 3000)
                        x, y = int(lm.x * (x2-x1)), int(lm.y * (y2-y1))
                        face_2d.append([x, y])
                        face_3d.append([x, y, lm.z])

                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)

                focal_length = 1 * (x2-x1)
                cam_matrix = np.array([[focal_length, 0, (y2-y1) / 2],
                                       [0, focal_length, (x2-x1) / 2],
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
                    torso_vector = calculate_torso_vector(results.pose_landmarks)
                    
                    if is_facing_camera(torso_vector):
                        label = "Body facing camera"
                        if face_text == "Looking Forward":
                            attentive = "Attentive"
                    else:
                        label = "Body not facing camera"
                    
                    cv2.putText(image, label, (x1 + 20, y1 + 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

                nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)
                p1 = (int(nose_2d[0]) + x1, int(nose_2d[1]) + y1)
                p2 = (int(nose_2d[0] + y * 10) + x1, int(nose_2d[1] - x * 10) + y1)

                cv2.line(image, p1, p2, (255, 0, 0), 3)
                cv2.putText(image, face_text, (x1 + 20, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(image, "x: " + str(np.round(x, 2)), (x1 + 500, y1 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(image, "y: " + str(np.round(y, 2)), (x1 + 500, y1 + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(image, "z: " + str(np.round(z, 2)), (x1 + 500, y1 + 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            cv2.putText(image, attentive, (x1 + 20, y1 + 300), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
            
            if results.face_landmarks:
                mp_drawing.draw_landmarks(image=image[y1:y2, x1:x2],
                                          landmark_list=results.face_landmarks,
                                          connections=mp_holistic.FACEMESH_CONTOURS,
                                          landmark_drawing_spec=drawing_spec,
                                          connection_drawing_spec=drawing_spec)

            if results.pose_landmarks:
                mp_drawing.draw_landmarks(image=image[y1:y2, x1:x2],
                                          landmark_list=results.pose_landmarks,
                                          connections=mp_holistic.POSE_CONNECTIONS)

    end = time.time()
    totalTime = end - start
    fps = 1 / totalTime
    print("FPS: ", fps)

    cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow('Head and Body Pose Detection', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
