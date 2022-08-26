# How to use the ROS2 pointcloud projection code

## Environment setup

```bash
sudo apt install ros-galactic-pcl-ros
source /opt/ros/galactic/setup.bash
```

## Navigating to the repo folder and launching ros node

```bash
cd ~/project_cloud
colcon build
source ./install/setup.bash

ros2 run livox_pcd_projection projection
```

## What does this code do? 

1. Reads the camera intrinsic data and correct camera distortion
2. Reads the extrinsic calibration data and project the pointcloud onto the image frame
3. Subscribe to `/camera/image_0` 
4. Subscribe to `BoundingBox2D`
5. Query and publish the average distance to the points in the bounding box to topic `/topic`