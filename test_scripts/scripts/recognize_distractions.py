import torch
import cv2
import numpy as np
import mediapipe as mp

model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

image_path = '/home/vboxuser/Desktop/Smart-Robotics-Project/test_scripts/3quarti.jpeg'
image = cv2.imread(image_path)

if image is None:
    print("Error loading image")
    exit()

results = model(image)
detections = results.pandas().xyxy[0]

person_detections = detections[detections['name'] == 'person']

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

def get_head_orientation(landmarks):
    nose_tip = landmarks[0]
    left_eye_inner = landmarks[1]
    left_eye_outer = landmarks[3]
    right_eye_inner = landmarks[4]
    right_eye_outer = landmarks[6]

    center_of_eyes = np.array([(left_eye_inner.x + right_eye_inner.x) / 2, (left_eye_inner.y + right_eye_inner.y) / 2])
    face_vector = np.array([nose_tip.x, nose_tip.y]) - center_of_eyes
    left_right_vector = np.array([right_eye_outer.x - left_eye_outer.x, right_eye_outer.y - left_eye_outer.y])

    return face_vector, left_right_vector

def is_attentive(face_vector, left_right_vector):
    try:
        threshold = 0.4
        attention_score = np.linalg.norm(face_vector) + np.linalg.norm(left_right_vector)
        return attention_score < threshold
    except Exception as e:
        print("Error in attention calculation:", e)
        return False

def put_text_on_image(image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=1, color=(0, 255, 0), thickness=2):
    cv2.putText(image, text, position, font, font_scale, color, thickness, cv2.LINE_AA)

with mp_pose.Pose(static_image_mode=True, min_detection_confidence=0.5) as pose:
    for index, person in person_detections.iterrows():
        xmin, ymin, xmax, ymax = int(person['xmin']), int(person['ymin']), int(person['xmax']), int(person['ymax'])
        
        person_roi = image[ymin:ymax, xmin:xmax]
        rgb_roi = cv2.cvtColor(person_roi, cv2.COLOR_BGR2RGB)
        rgb_roi.flags.writeable = False
        
        results = pose.process(rgb_roi)
        
        rgb_roi.flags.writeable = True
        bgr_roi = cv2.cvtColor(rgb_roi, cv2.COLOR_RGB2BGR)
        
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(
                bgr_roi,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                mp_drawing.DrawingSpec(color=(245, 23, 15), thickness=1, circle_radius=1),
                mp_drawing.DrawingSpec(color=(245, 107, 15), thickness=1, circle_radius=1)
            )
            
            face_vector, left_right_vector = get_head_orientation(results.pose_landmarks.landmark)
            attentive = is_attentive(face_vector, left_right_vector)
            
            label = "Attentive" if attentive else "Distracted"
            put_text_on_image(image, label, (xmin, ymin - 10), font_scale=0.5, thickness=1)

        image[ymin:ymax, xmin:xmax] = bgr_roi

output_path = '/home/vboxuser/Desktop/Smart-Robotics-Project/test_scripts/output_image.jpg'
cv2.imwrite(output_path, image)
print(f'Processed image saved to {output_path}')
