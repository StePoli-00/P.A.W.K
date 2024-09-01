import numpy as np
import cv2
import mediapipe as mp
import time

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


# Initialize holistic model
mp_holistic = mp.solutions.holistic
holistic = mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Drawing utilities
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(color=(128, 0, 128), thickness=1, circle_radius=1)

# Open video capture
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    start = time.time()
    
    # Flip the image for a selfie-view display
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    
    # Process the image and find the face and body landmarks
    results = holistic.process(image)
    
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    img_h, img_w, img_c = image.shape
    face_2d = []
    face_3d = []

    if results.face_landmarks:
        attentive = "Not attentive"
        for idx, lm in enumerate(results.face_landmarks.landmark):
            if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
                if idx == 1:
                    nose_2d = (lm.x * img_w, lm.y * img_h)
                    nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)
                x, y = int(lm.x * img_w), int(lm.y * img_h)
                face_2d.append([x, y])
                face_3d.append([x, y, lm.z])

        # Get 2D and 3D coordinates
        face_2d = np.array(face_2d, dtype=np.float64)
        face_3d = np.array(face_3d, dtype=np.float64)

        focal_length = 1 * img_w
        cam_matrix = np.array([[focal_length, 0, img_h / 2],
                               [0, focal_length, img_w / 2],
                               [0, 0, 1]])
        distortion_matrix = np.zeros((4, 1), dtype=np.float64)

        success, rotation_vec, translation_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, distortion_matrix)

        # Get rotational matrix and angles
        rmat, jac = cv2.Rodrigues(rotation_vec)
        angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

        x = angles[0] * 360
        y = angles[1] * 360
        z = angles[2] * 360

        # Determine face orientation
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

        # Body orientation detection
        if results.pose_landmarks:
            torso_vector = calculate_torso_vector(results.pose_landmarks)
                
            if is_facing_camera(torso_vector):
                label = "Body facing camera"
                if face_text == "Looking Forward":
                    attentive = "Attentive"
            else:
                label = "Body not facing camera"
                
            # Visualizza il risultato sull'immagine
            cv2.putText(image, label, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # Nose direction line
        nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rotation_vec, translation_vec, cam_matrix, distortion_matrix)
        p1 = (int(nose_2d[0]), int(nose_2d[1]))
        p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))

        cv2.line(image, p1, p2, (255, 0, 0), 3)
        cv2.putText(image, face_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(image, "x: " + str(np.round(x, 2)), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(image, "y: " + str(np.round(y, 2)), (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(image, "z: " + str(np.round(z, 2)), (500, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    end = time.time()
    totalTime = end - start
    fps = 1 / totalTime
    print("FPS: ", fps)

    cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(image, attentive, (20, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
    if results.face_landmarks:
        mp_drawing.draw_landmarks(image=image,
                                  landmark_list=results.face_landmarks,
                                  connections=mp_holistic.FACEMESH_CONTOURS,
                                  landmark_drawing_spec=drawing_spec,
                                  connection_drawing_spec=drawing_spec)

    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image=image, landmark_list=results.pose_landmarks, connections=mp_holistic.POSE_CONNECTIONS)

    cv2.imshow('Head and Body Pose Detection', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
