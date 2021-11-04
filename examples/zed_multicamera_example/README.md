# Stereolabs ZED Camera - ROS Multi Camera example

`zed_multicamera_example` is a ROS package to illustrate how to use multiple ZED cameras in the same environment

## Installation

Follow the [install guide](https://github.com/stereolabs/zed-ros-examples/tree/master/README.md)

## Run the program

Three examples are provided:
* `zed_multi_cam.launch`: starts two cameras nodes in different processes.
* `zed_multi_gpu.launch`: illustrates how to assign different process to different GPUs in case the host has multiple available GPUs.
* `zed_multicam_single_nodelet.launch`: starts two camera nodelets in the same nodelet manager. This is the most complete example, allowing to associate camera node names to the correct camera by using the serial number.





