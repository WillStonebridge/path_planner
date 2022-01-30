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
  source ~/catkin_ws/src/path_planner/launch-common.sh
  ```
2. Run QGC or Mavproxy
  - QGC: Run QGC, through some magic it should connect
  - Mavproxy: 
    1. Download and install Mavproxy in Docker: https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html
    ```bash
    sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
    pip3 install PyYAML mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
    ```
    2. Run and connect Mavproxy to SITL
    ```bash
    mavproxy.py --master localhost:14550
    ```
3. Launch mission_mode.launch (currently only launches px4_sitl)
  ```bash
  roslaunch mission_mode mission_mode.launch
  ```
4. Run mission_mode_node
```bash
rosrun mission_mode mission_mode_node 
```

General Notes:
interop json altitude should be in feet
resolution should be in meters
buffer should be in meters
