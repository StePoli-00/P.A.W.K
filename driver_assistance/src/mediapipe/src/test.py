import mediapipe as mp
import cv2
import numpy as np

cap = cv2.VideoCapture(0)
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
#check if webcam is working 
def check_webcam():
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if ret==True:
            cv2.imshow('Mediapipe Feed', frame)
            
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break
        else:
            print("Acquisition Failed")
            break
            
    cap.release()
    cv2.destroyAllWindows()
    return 


print(mp_pose.POSE_CONNECTIONS)

## Setup Mediapipe instance
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        ret, frame = cap.read()
        
        # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        
        #Mirror the webcam
        image=cv2.flip(image,1)
        # Make detection
        results = pose.process(image)

        # Recolor back to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        # Render detections
        mp_drawing.draw_landmarks(image, 
                                  results.pose_landmarks, #landmark list
                                  mp_pose.POSE_CONNECTIONS, #connection lists
                                mp_drawing.DrawingSpec(color=(245,23,15), #keypoint color in BGR
                                                       thickness=2, circle_radius=2), 
                                mp_drawing.DrawingSpec(color=(245,107,15), #connection color  in BGR
                                                       thickness=2, circle_radius=2) 
                                 )               
        
        cv2.imshow('Mediapipe Feed', image)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
