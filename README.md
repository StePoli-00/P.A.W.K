# Smart Robotics Project
This repo contains code of Smart Robotics Project

# Setup

## Clone Project & Create Conda Environment
```sh
git clone https://github.com/StePoli-00/Smart-Robotics-Project.git
cd Smart-Robotics-Project
conda create --name <env_name>
conda activate <env_name>
pip install -r requirements.txt
```
## Build the project
> [!NOTE]
> the environment must be compiled with python3 for the first time with python3 
```sh
cd pedestrian_detection
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```





