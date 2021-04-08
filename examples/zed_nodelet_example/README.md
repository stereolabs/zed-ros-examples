# Stereolabs ZED Camera - ROS Nodelet example

`zed_nodelet_example` is a ROS package to illustrate how to load the `ZEDWrapperNodelet` with an external nodelet manager and use the intraprocess communication exploiting zero-copy.

Two examples are provided:
- topics synchronization and mux
- depth image to 2D laser scan

## Installation

Follow the [install guide](https://github.com/stereolabs/zed-ros-examples/blob/master/README.md#build-the-program)

**Note:** ROS Noetic does not include a precompiled version of the `depthimage-to-laserscan` package. You must compile it from source, the `melodic-devel` branch is fully compatible:

```
$ cd <catkin_root>/src
$ git clone https://github.com/ros-perception/depthimage_to_laserscan.git --branch melodic-devel
$ cd ..
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Usage
A full explanation about how to start the two examples is provided on the [official Stereolabs online documentation](https://www.stereolabs.com/docs/ros/zed_nodelets/#examples)



