import torch
import cv2 
import rospy
model = torch.hub.load('ultralytics/yolov5', 'yolov5n')
image_path="/home/stefano/Desktop/Smart-Robotics-Project/test_scripts/2_people.png"
image = cv2.imread(image_path)

if image is None:
    print("error")
   

results = model(image)