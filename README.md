## 1. Environment
- PX4 SITL
- Ubuntu 20.04LTS with VMWare
- ROS Noetic

## 2. How to use?
1. Clone this project to your catkin workspace
   ```bash
   mkdir ~/catkin_ws/src
   cd catkin_ws/src
   git clone https://github.com/EelKor/UASG-Offboard.git
   ```
2. Build Catkin_ws
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
4. Launch mavros_posix_sitl
   ```bash
   roslaunch px4 mavros_posix_sitl.launch
   ```

5. Launch python node
   ```bash
   cd ~/catkin_ws
   source devel/setup.bash
   rosrun offb_py offb.py
   ```
   **offb_py** is package name and **offb.py** is script name.
   Name can be changed
   default name is same to example above


## REFERENCE
   1. https://github.com/nkhedekar/offb_py
   2. https://www.youtube.com/watch?v=BLH0iyANl1I, Accessed on Jan 9, 2024
   
