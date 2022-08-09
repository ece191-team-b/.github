# How to launch lidar + camera and collect data using rosbag

- [How to launch lidar + camera and collect data using rosbag](#how-to-launch-lidar--camera-and-collect-data-using-rosbag)
  - [How to collect data using `rosbag`](#how-to-collect-data-using-rosbag)
    - [Launch the lidars](#launch-the-lidars)
    - [Launch cameras](#launch-cameras)
    - [Record the data using rosbag](#record-the-data-using-rosbag)
  - [Misc Troubleshooting notes](#misc-troubleshooting-notes)
    - [Calibrate the camera using NAV 2](#calibrate-the-camera-using-nav-2)

## How to collect data using `rosbag`

### Launch the lidars

Build and source the Livox ROS 2 Driver

```bash
source /opt/ros/galactic/setup.bsah
cd ~/ws_livox
colcon build
source ./install/setup.bash
```

Launch Livox lidars using provided launch files

```bash
cd ~/ws_livox/src/livox_ros2_driver/launch
ros2 launch livox_lidar_rviz_launch.py
```

This should launch rviz with the point clouds + camera frames visualized.

The rviz config file is stored at

```bash
~/ws_livox/src/livox_ros2_driver/config/livox_lidar.rviz
```

### Launch cameras

Open another terminal:

```bash
source /opt/ros/galactic/setup.bsah
cd ~/multi_cam_oak_lite
source ./install/setup.bash
ros2 run multi_cam_obj_detection cams
```

If encounter error related to USB, try: 

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

In another terminal: check that the topics are being published

```bash
ros2 topic list
```

We're looking for:

```bash
/livox/lidar_3WEDH7600103311
/livox/lidar_3WEDH7600104801
/camera/image_0
/camera/image_1
```

If all 4 of these are present, you're good to go.

### Record the data using rosbag

```bash
cd ~/
mkdir bag_files && cd ~/bag_files # create and navigate to directory where bag files will be stored
ros2 bag record /livox/lidar_3WEDH7600104801 /livox/lidar_3WEDH7600103311 /camera/image_0 /camera/image_1
```

Note: the size of the bag file might become quite big. If a recording is not needed, it is probably a good idea to delete them.

## Misc Troubleshooting notes

**Ignore this section if no issues were encountered**
Installing depthai on Jetson Xavier
<https://docs.luxonis.com/projects/api/en/latest/install/#jetson>

### Calibrate the camera using NAV 2

<https://navigation.ros.org/tutorials/docs/camera_calibration.html>

```bash
ros2 run camera_calibration cameracalibrator --size 6x6 --square 0.02 --pattern 'chessboard' --ros-args -r image:=/camera/image_0
ros2 run camera_calibration cameracalibrator --size 6x6 --square 0.02 --pattern 'chessboard' --ros-args -r image:=/camera/image_1
```

Common errors:

- using `python` instead of `python3`

