import torch
import cv2
import numpy as np
import mediapipe as mp

model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

image_path = '/home/vboxuser/Desktop/Smart-Robotics-Project/test_scripts/2_people.png' 
image = cv2.imread(image_path)

if image is None:
    print("Error loading image")
    exit()

results = model(image)
detections = results.pandas().xyxy[0]

person_detections = detections[detections['name'] == 'person']

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
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
                mp_drawing.DrawingSpec(color=(245, 23, 15), thickness=2, circle_radius=2), 
                mp_drawing.DrawingSpec(color=(245, 107, 15), thickness=2, circle_radius=2)
            )
        image[ymin:ymax, xmin:xmax] = bgr_roi

cv2.imshow('Pose Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
