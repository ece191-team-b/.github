# How to launch lidar + camera and collect data using rosbag

- [How to launch lidar + camera and collect data using rosbag](#how-to-launch-lidar--camera-and-collect-data-using-rosbag)
  - [How to collect data using `rosbag`](#how-to-collect-data-using-rosbag)
    - [Launch the Livox Horizon lidars](#launch-the-livox-horizon-lidars)
    - [Launch OAK-D lite cameras](#launch-oak-d-lite-cameras)
    - [Record the data using rosbag](#record-the-data-using-rosbag)
  - [Misc Troubleshooting notes](#misc-troubleshooting-notes)
    - [Calibrate the camera using NAV 2](#calibrate-the-camera-using-nav-2)

## How to collect data using `rosbag`

### Launch the Livox Horizon lidars

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

This should launch rviz with the point clouds frames visualized.

The rviz config file is stored at

```bash
~/ws_livox/src/livox_ros2_driver/config/livox_lidar.rviz
```

### Launch OAK-D lite cameras

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

To double check: in a different ros2 sourced terminal: 

```bash
rviz2
```

and then select the /camera/image_0 topic using the `add` button on the left panel.

If there is no image shown, there can be a few possible reasons:

1. The camera may be disconnected
2. Colcon build failed / didn't overwrite previous build
3. Debug mode for the `multi_cams cams` ros node is on (python script is located at `/multi_cam/multi_cam_node.py`)

By default in `multi_cam_node.py` debug mode should be off, if it is set to on you should see something like the following in the `main()` function: 

```python
def main():
    rclpy.init()
    cam_node = MultiCamNode()
    cam_node.camera_initialization(debug=True, path ='some_path_to_saved_imgs')
```

If `debug=False` or no extra arguments are passed to `cam_node.camera_initialization()` then debug mode is off.

We're looking for:

```bash
/livox/lidar_3WEDH7600103311
/livox/lidar_3WEDH7600104801
/camera/image_0
/camera/image_1
```

If all 4 of these are present, you're good to go.

Note: If you are using a different livox horizon lidar, you may see a different serial number than `3WEDH7600103311` or `3WEDH7600104801`.

### Record the data using rosbag

Change the name of the topics to record based on what you saw with `ros2 topic list`.

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