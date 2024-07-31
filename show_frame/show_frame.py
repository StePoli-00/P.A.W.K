import numpy as np
import re
import yaml
import ast
import json
import pickle
import os
import uuid
import cv2
folder_path="/home/stefano/Desktop/Smart-Robotics-Project/show_frame"
input_file="/home/stefano/Desktop/Smart-Robotics-Project/show_frame/image.txt"
class FileFrame():
    def __init__(self,data, header=None):
        width,height,encoding,data,header=self.get_data(data)
        self.width=width
        self.height=height
        self.encoding=encoding
        self.header=header
        self.data=data

    def get_data(self,data):
        byte_data=data[len(data)-1]
        string_data="".join(data[0:len(data)-1])
        yaml_data=yaml.safe_load(string_data)
        _,byte_data=byte_data.split("data: ")
        byte_data=byte_data.replace("\n","")
        byte_data=byte_data.strip('[]')
        byte_data= [int(x) for x in byte_data.split(',')]
       
        return yaml_data.pop("width"),yaml_data.pop("height"),yaml_data.pop("encoding"),byte_data,yaml_data
    
    def convert_into_images(self):
        
        image=np.array(self.data,dtype=np.uint8)
        if self.encoding=="rgb8":
            image=image.reshape((self.height,self.width,3))
            
        elif self.encoding=="mono8":
            image=image.reshape((self.height,self.width))
        
        return image

def save_file_frame(folder_path):
    with open(os.path.join(folder_path,"file_frames.pickle"),"wb") as f:
        pickle.dump(frames,f)


def load_frame_from_file(file_path):
    frames=[]
    with open(file_path,"r") as f:
        content=[]
        while(1):
            data=f.readline()
            if not data:
                break
            if "---" in data:
                frames.append(FileFrame(content))  
                content.clear()
              
            content.append(data)
        
    return frames

def save_image(image,out_file):
    cv2.imwrite(os.path.join(folder_path,out_file),image)
    return





if __name__=="__main__":
    frames=load_frame_from_file(input_file)
    images=[]
    for i,frame in enumerate(frames):
        img=frame.convert_into_images()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imshow("frame",img)
        save_image(img,f"frame{i}.png")
        cv2.waitKey(0)

    cv2.destroyAllWindows()