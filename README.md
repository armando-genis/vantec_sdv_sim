# vantec_sdv_sim
 
## Install dependencies:

```bash
sudo apt-get install libeigen3-dev
sudo apt install libeigen3-dev
sudo apt install ros-<ros2-distro>-joint-state-publisher-gui
sudo apt install ros-<ros2-distro>-xacro
sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs
sudo apt install ros-<ros2-distro>-ackermann-msgs
sudo apt install ros-<ros2-distro>-ros2-control ros-<ros2-distro>-ros2-controllers
sudo apt install ros-<ros2-distro>-controller-manager
```


## Run it:

```bash
source /opt/ros/foxy/setup.bash #for ros2 foxy
source /opt/ros/humble/setup.bash #for ro2 humble
source install/setup.bash
ros2 launch sdv_description gazebo_final.launch.py
```

