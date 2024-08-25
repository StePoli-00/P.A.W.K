# Smart Robotics Project
This Repository contains the code of Smart Robotics Project

# Setup

## Clone Project
```sh
git clone https://github.com/StePoli-00/Smart-Robotics-Project.git
cd Smart-Robotics-Project
```
 ## Create Conda Environment
```sh
conda create --name <env_name>
conda activate <env_name>
pip install -r requirements.txt
conda install python (our version is 3.12)
```
## Build the project
> [!NOTE]
> the environment must be compiled with python3 for the correct dependencies installarion
```sh
cd pedestrian_detection
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## Add catkin_ws into repo



