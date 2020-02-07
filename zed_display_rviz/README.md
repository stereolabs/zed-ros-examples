# Stereolabs ZED Camera - ROS Display package

This package lets you visualize in the [ROS RViz application](http://wiki.ros.org/rviz) all the
possible information that can be acquired using a Stereolabs camera.
The package provides the launch files for ZED, ZED Mini and ZED 2 camera models.

**Note:** The main package [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)
is required to correctly execute the ROS node to acquire data from a Stereolabs 3D camera.

## Getting started

   - First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper/#build-the-program)
   - [Install](#Installation) the package
   - Read the online documentation for [More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

### Prerequisites

   - Ubuntu 16.04 or newer (Ubuntu 18 recommended)
   - [ZED SDK **≥ 3.0**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
   - [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Installation

The *zed_display_rviz* is a catkin package. It depends on the following ROS packages:
  - rviz
  - rviz_imu_plugin
  - zed_wrapper

Install the [zed-ros-wrapper](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) package
following the [installation guide](https://github.com/stereolabs/zed-ros-wrapper#build-the-program)

Install the [zed-ros-examples](https://github.com/stereolabs/zed-ros-examples) package following the [installation guide](https://github.com/stereolabs/zed-ros-examples#build-the-program)

### Execution

If you own a ZED camera launch:

    $ roslaunch zed_display display_zed.launch

If you own a ZED Mini camera launch:

    $ roslaunch zed_display display_zedm.launch

If you own a ZED 2 camera launch:

    $ roslaunch zed_display display_zed2.launch

![ZED rendering on Rviz](images/depthcloud-RGB.jpg)
![ZED rendering on Rviz](images/ZEDM-Rviz.jpg)
![ZED rendering on Rviz](images/ZED-Rviz.jpg)

[Detailed information](https://www.stereolabs.com/docs/ros/rviz/)
