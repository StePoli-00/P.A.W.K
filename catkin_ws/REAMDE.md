1. 
```sh
vcs import --input \
 https://raw.githubusercontent.com/RobotnikAutomation/rbkairos_sim/melodic-devel/repos/rbkairos_sim.repos
```
2. rosdep install --from-paths src --ignore-src -y -r
3. catkin build


For errors with catkin build
remove `.catkin_tools` folder