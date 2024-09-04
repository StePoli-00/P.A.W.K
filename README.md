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
> BUILD ALWAYS with catkin_make specify python3
```sh
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
cd pedestrian_detection

```


### catkin_ws
```sh
cd catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```
## Run collision detection algorithm

1. launch the simulation
```sh
roslaunch rbkairos_sim_bringup rbkairos_complete.launch
```
2. run rosnode to detect static and dynamic obstacles
 ```sh
rosrun obstacle_detection new_version.py
```
3. run control algorithm 
```sh
rosrun mpc_algorithm algorithm_<version>.py
```





