![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS Tutorials and Examples

This package is a collection of examples and tutorials to illustrate how to better use the ZED cameras in the ROS framework

[More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

## Getting started

- First, be sure to have installed the main ROS package to integrate the ZED cameras in the ROS framework: [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper/)
- [Install](#build-the-program) the Tutorials package
- Read the online documentation for [More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

### Prerequisites

- Ubuntu 16.04 or newer (Ubuntu 18.04 recommended)
- [ZED SDK **≥ 3.0**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (recommended)

### Build the program

The `zed-ros-examples` repository is a collection of catkin packages. They depend on the following ROS packages:

   - catkin
   - zed_wrapper
   - sensor_msgs
   - roscpp
   - nav_msgs
   - geometry_msgs
   - ar_track_alvar
   - ar_track_alvar_msgs
   - nodelet
   - depthimage_to_laserscan
   - rtabmap
   - rtabmap_ros
   - rviz_imu_plugin
   - rviz
   - plotjuggler

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ cd ~/catkin_ws/src
    $ git clone https://github.com/stereolabs/zed-ros-examples.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -r -y
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    $ source ./devel/setup.bash

## Run the tutorials and examples

### Rviz visualization examples

 - Example launch files to start a preconfigured instance of Rviz displaying all the ZED Wrapper node information: [zed_display_rviz](https://github.com/stereolabs/zed-ros-examples/tree/master/zed_display_rviz/README.md)

### Tutorials

 - [Image subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_video_sub_tutorial/README.md)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_depth_sub_tutorial/README.md)
 - [Tracking subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_tracking_sub_tutorial/README.md) 
 - [Sensors data subscription tutorial](https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_sensors_sub_tutorial/README.md) 
 - [Object detection subscription tutorial](https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_obj_det_sub_tutorial/README.md) 

### Examples

 - [AR Track Alvar example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_ar_track_alvar_example/README.md) 
 - [Nodelet example (point cloud to laser scan)](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_nodelet_example/README.md) 
 - [RTABmap example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_rtabmap_example/README.md) 



