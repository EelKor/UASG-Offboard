## 1. Environment

- Ubuntu 20.04LTS with VMWare
- ROS Noetic

## 2. How to use?
1. Clone this project to your catkin workspace
   ```bash
   mkdir ~/catkin_ws/src
   cd catkin_ws/src
   git clone
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
   offb_py is package name and offb.py is script name.
   Name is depend on you!
   default name is same to example above
   
