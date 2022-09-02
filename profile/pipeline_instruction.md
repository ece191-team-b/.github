# UCSD ECE191 Summer 22' TeamB Perception Instruction Overview

## Table of contents 
- [UCSD ECE191 Summer 22' TeamB Perception Instruction Overview](#ucsd-ece191-summer-22-teamb-perception-instruction-overview)
  - [Table of contents](#table-of-contents)
  - [Installation](#installation)
    - [Install ROS galactic](#install-ros-galactic)
    - [Install colcon](#install-colcon)
    - [Livox ROS2 driver](#livox-ros2-driver)
    - [Livox pointcloud projection](#livox-pointcloud-projection)
    - [OAK-D lite](#oak-d-lite)
    - [Pytorch](#pytorch)
  - [Camera Lidar Calibration](#camera-lidar-calibration)
    - [ROS Noetic installtion](#ros-noetic-installtion)
  - [Lidar Projection](#lidar-projection)
  - [Cameras](#cameras)

## Installation 
This Project was primarily built to run on Jetson AGX Xavier with Ubuntu 20.04, as most instruction are compatiable with other Ubuntu Machine. 

### Install ROS galactic

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

### Install colcon

```bash
sudo apt install python3-colcon-common-extensions
```

### Livox ROS2 driver 

```bash
git clone https://github.com/ece191-team-b/livox_ros2_driver ws_livox/src
cd ws_livox
source /opt/ros/galactic/setup.bash
colcon build
source ./install/setup.bash
```

After the setup in complete, in the same terminal you can launch the lidars using the launch files

Navigate to the directory where all the launch files are contained

```bash
cd ~/ws_livox/src/livox_ros2_driver/launch
```

Select a launch file to launch the lidars with. Usually, we use `livox_lidar_msg_launch.py`. This makes the lidar publish livox's custom message as detailed [here](https://github.com/ece191-team-b/livox_ros2_driver#41-launch-file-configuration-instructions) in the documentation.

```bash
ros2 launch livox_lidar_msg_launch.py 
```

Alternatively, if you would like to launch a visualization tool, such as `rviz2`, launch it with 

```bash
ros2 launch livox_lidar_rviz_launch.py
```

To learn more about what each launch file differs from each other and how to customize the parameters, please refer to the [documentation](https://github.com/ece191-team-b/livox_ros2_driver#41-launch-file-configuration-instructions).

If you are using our fork of `livox_ros2_driver` it should come with modifications to these launch files that are suitable for the perception pipeline. 

### [Livox pointcloud projection](https://github.com/ece191-team-b/livox_projection)

```bash
cd ~
git clone https://github.com/ece191-team-b/livox_projection
cd ~/livox_projection
```

Install dependencies `pcl-ros`. If you have gone through all the installations for the other packages above, this should be the only dependency that is unique to this package. 

```bash
sudo apt install ros-galactic-pcl-ros
```

Build the workspace with the following commands

```bash
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
source ./install/setup.bash
```

Place the calibration results in the following file path. A default in/extrinsic calibration file should already be provided. If the relative position of the camera and lidar pair are changed, a new calibration would be needed. 

```bash
~/livox_projection/calibration_data/parameters/intrinsic.txt
~/livox_projection/calibration_data/parameters/extrinsic.txt
```

Launching the stamper and projector node

In the first terminal, launch the stamper node: 

```bash
cd ~/livox_projection
ros2 launch livox_pcd_projection stamper.launch.py
```

In a second (new) terminal, launch the projector node:

```bash
cd ~/livox_projection
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
ros2 run livox_pcd_projection projector.launch.py
```

If there are no error messages from either node and all the camera and lidar nodes are launched you should be able to do `ros2 topic echo /distances` and see the lidar distance reading of each detected objects.

### OAK-D lite

```bash
pip install depthai
source /opt/ros/galactic/setup.bash
git clone https://github.com/Triton-AI/multi_cam_oak_lite
cd multi_cam_oak_lite
colcon build
source ./install/setup.bash
```

Then, run the following command:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Pytorch 
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
## Camera Lidar Calibration
**Note: Camera Lidar Calibration is done in ROS noetic instead of ROS2 Galactic**
Additionally, the camera lidar calibration does **not** need to be the run on the AGX. Installing on a personal ubuntu machine with all the required packages installed can work as well. 

When the calibration is complete, simply transfer the calibration data over to the AGX at the directory specified in the [livox_projection](https://github.com/ece191-team-b/livox_projection#124-calibration-results) package documentation.

### ROS Noetic installtion
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

**Note**: To do calibration, there will be **TWO** terminals open, one in **_ROS noetic_ to record the rosbag of the lidar**, and the other will be our **Camera node in _ROS2 Galactic_.** Please only connect **ONE PAIR** of camera and lidar at a time for calibration. Details for launching cameras can be found at [here](#cameras).

**Please then go to this [repo Link](https://github.com/ece191-team-b/livox_camera_lidar_calibration) and follow the readme to calibration camera and lidar.**

## Lidar Projection

Please refer to this [repo](https://github.com/ece191-team-b/livox_projection/blob/master/README.md) for usage of the projection node.


## Cameras 

Please refer to this [repo](https://github.com/ece191-team-b/multi_cam_detection) for further instruction.