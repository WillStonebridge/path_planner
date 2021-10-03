# PathFinding
Astar Algorithm for PARTIEEE

A* algorithm planner and Mapping classes for PARTIEEE

Reference http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html for implementation and theory

# Getting Started

1. Pull down the latest docker image (info can be found in the drive) and follow the directions

1. (In your Docker Container) Clone this repository in your catkin workspace 
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/PurdueAerialRoboticsTeam/path_planner.git
   ```
1. Build the mission_mode node.

   ```bash
   source /opt/ros/melodic/setup.bash
   cd ~/catkin_ws
   catkin build 
   ```
1. Source the catkin setup.bash from your catkin workspace:
   ```bash   
   source ~/catkin_ws/devel/setup.bash
   ```
# Running mission_mode node
1. Update ROS Environment for Gazebo ROS wrappers 
  ```bash
  source ~/catkin_ws/src/path_planner/launch/launch-common.sh
  ```
1. Run QGC or Mavproxy
  - QGC: Run QGC, through some magic it should connect
  - Mavproxy: 
    ```bash
    mavproxy.py --master localhost:14550
    ```
3. Launch mission_mode.launch
  ```bash
  roslaunch mission_mode mission_mode.launch
  ```
