# Setting up the AGX

## Install ROS galactic

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

## Install colcon

```bash
sudo apt install python3-colcon-common-extensions
```

## Livox

```bash
git clone https://github.com/ece191-team-b/livox_ros2_driver ws_livox/src
cd ws_livox
source /opt/ros/galactic/setup.bash
colcon build
source ./install/setup.bash
```

## OAK-D lite

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
