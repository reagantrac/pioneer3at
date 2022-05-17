# Pioneer3at control via ROS
Work in progress.

## 1. ROS Installation
1.1 Must use Ubuntu 20.04 (VM is Ok), need plenty of space (approx. 4GB not including OS)

1.2 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

1.3 sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

1.4 sudo apt update

1.5 sudo apt install ros-noetic-desktop-full

1.6 sudo apt install python3-rosdep

1.7 sudo rosdep init

1.8 rosdep update

## 2. Environment Setup
2.1 echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

2.2 source ~/.bashrc

2.3 mkdir -p ~/catkin_ws/src

2.4 cd ~/catkin_ws/src/

2.5 git clone https://github.com/reagantrac/pioneer3at

## 3. Running ROS nodes
TODO
