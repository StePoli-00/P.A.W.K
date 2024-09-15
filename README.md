# Smart Robotics Project
This Repository contains the code of Smart Robotics Project.
This project aim to have an obstacle avoidance control analizing actors attention on Robotnik Kairos simulator, implemented using a MPC algorithm. 

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
## Run simualtor and obstacle avoiding algorithm

1. Just running the simulation, all the required node will be instantiated 
```sh
roslaunch rbkairos_sim_bringup rbkairos_complete.launch
```
## Useful Nodes
1. Pedestrian Attention Detection - camera screen to visualize body and face landmarks and assinged YOLO label to an actor. 
```sh
rosrun multi_det attention_subscriber.py
```
## Load actor model
>[NOTE]
1. place `/Smart-Robotics-Project/catkin_ws/src/rbkairos_sim/rbkairos_gazebo/worlds/business_man` inside `<username>/home/.gazebo/models` CTRL+H to see hiding files









