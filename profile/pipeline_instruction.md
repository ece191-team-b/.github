# UCSD ECE191 Summer 22' TeamB Perception Instruction Overview

## Table of contents 
* [Installation](#Installation)
   * [Setting up AGX](#Setting-Up-AGX)
   * [Camera-Lidar Calibration](#camera-lidar-calibration)

## Installation 
This Project was primarily built to run on Jetson AGX Xavier with Ubuntu 20.04, as most instruction are compatiable with Other Ubuntu Machine. 

### Setting Up AGX
#### Install ROS galactic

Follow the offical documentation's instruction
<https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html>

Also provided here for your convenience:

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

Enable the Universe repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS 2 apt repository to your system. First authorize ROS' GPG key with apt.

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add ROS repo to sources list

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Ensure system is upto date before installing

```bash
sudo apt update
sudo apt upgrade
```

Install ROS, RViz, demos, tutorials.

```bash
sudo apt install ros-galactic-desktop
```

#### Install colcon

```bash
sudo apt install python3-colcon-common-extensions
```

#### Livox

```bash
git clone https://github.com/ece191-team-b/livox_ros2_driver ws_livox/src
cd ws_livox
source /opt/ros/galactic/setup.bash
colcon build
source ./install/setup.bash
```

#### OAK-D lite

```bash
pip install depthai
source /opt/ros/galactic/setup.bash
git clone https://github.com/Triton-AI/multi_cam_oak_lite
cd multi_cam_oak_lite
colcon build
source ./install/setup.bash
```

You probably need to run this:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### Pytorch 
Please first try to see is torch is installed and cuda is avabliable.

```bash
Python3 
>>> Import torch
>>> torch.cuda.is_available()
True 
```
If **False** is returned, please do
```bash
pip uninstall torch
pip3 uninstall torch 
```
and follow [official document](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html) to install Pytorch. Please repeat the previous step to check if pytorch is using CUDA. Then we will install torchvision.

```bash
cd ~
git clone https://github.com/pytorch/vision
cd vision
sudo python setup.py install
```
### Camera Lidar Calibration
**Note: Camera Lidar Calibration is done in ROS noetic instead of ROS2 Galactic**
#### ROS Noetic installtion
Open up a terminal
```bash
# accept software from ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```
Open up a new terminal 
```bash
source /opt/ros/noetic/setup.bash
```

**_Note_: IMPORTANT!! If running catkin_make report OpenCV issues, please try to rebuild cv_bridge from source.**
```bash
sudo apt remove ros-noetic-cv-bridge
cd ~
git clone https://github.com/ros-perception/vision_opencv.git
git checkout noetic # make sure in the right branch
```
and **copy only the cv_bridge folder** to where livox lidar camera calibration is at and rebuild the whole thing again.

**Note**: To do calibration, there will be **TWO** terminals open, one in **_ROS noetic_ to record the rosbag of the lidar**, and the other will be our **Camera node in _ROS2 Galactic_.** Please only connect **ONE PAIR** of camera and lidar at a time for calibration. Details for lunching cameras can be found at [here](#cameras).

**Please then go to this [repo Link](https://github.com/ece191-team-b/livox_camera_lidar_calibration) and follow the readme to calibration camera and lidar.**

## Lidars 

# TODO BEN 


## Cameras 

Please go to this [repo link](https://github.com/ece191-team-b/multi_cam_detection) for further instruction.