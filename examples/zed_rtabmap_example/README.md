# Stereolabs ZED Camera - RTAB-map example

This package shows how to use the ZED Wrapper with [RTAB-map](http://introlab.github.io/rtabmap/)

## Installation

Follow the [install guide](https://github.com/stereolabs/zed-ros-examples/blob/master/README.md#build-the-program)

## Run the program

To launch the example, open a terminal and launch:

    $ roslaunch zed_rtabmap_example zed_rtabmap.launch

Example of indoor 3D mapping using RTAB-map and ZED

![Example of indoor 3D mapping](images/rtab-map.jpg)

## The launch file explained

To correctly use the ZED wrapper with the `rtabmap_ros` node we need to match the following RTABmap parameters:

- `rgb_topic` -> topic of the color information to be associated to the points of the 3D map
- `depth_topic` -> topic of the depth information
- `camera_info_topic` -> topic of RGB camera parameters used to create the association of each color pixel to the relative 3D point
- `depth_camera_info_topic` -> topic of the depth camera parameter, to convert the 2D depth image to a 3D point cloud
- `odom_topic` -> name of the camera frame

The values associated to the above parameters are the following:

```
    <arg name="odom_topic"              default="zed_node/odom" />
    <arg name="rgb_topic"               default="zed_node/rgb/image_rect_color" />
    <arg name="depth_topic"             default="zed_node/depth/depth_registered" />
    <arg name="camera_info_topic"       default="zed_node/rgb/camera_info" />
    <arg name="depth_camera_info_topic" default="zed_node/depth/camera_info" />
```

**Note**: the example as been tested using the packages `rtabmap v0.19.3` and `rtabmap_ros v0.19.3` available with the binary version of ROS Melodic. 
To check the version of RTABmap currently installed on your system you can use the commands:
`$ rosversion rtabmap`
and
`$ rosversion rtabmap_ros`



