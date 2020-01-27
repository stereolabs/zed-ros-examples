# Tutorials
A series of tutorials are provided to better understand how to use the ZED node in the ROS environment :

- [Video subscribing](./zed_video_sub_tutorial) : `zed_video_sub_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Image` to retrieve the Left and Right rectified images published by the ZED node

- [Depth subscribing](./zed_depth_sub_tutorial) : `zed_depth_sub_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Image` to retrieve the depth images published by the ZED node and to get the measured distance at the center of the image

- [Positional tracking subscribing](./zed_tracking_sub_tutorial) : `zed_tracking_sub_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `geometry_msgs/PoseStamped` and `nav_msgs/Odometry` to retrieve the position and the orientation of the ZED camera in the map and in the odometry frames.

- [Detected Object subscribing](./zed_obj_det_sub_tutorial) : `zed_obj_det_sub_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `zed_wrapper/objects` to retrieve the list of objects detected by a ZED 2 camera Object Detection module.

- [Sensors data subscribing](./zed_sensors_sub_tutorial) : `zed_sensors_sub_tutorial` - in this tutorial you will learn how to write a simple node that subscribes to messages of type `sensor_msgs/Imu`, `sensor_msgs/MagneticFiel`, `sensor_msgs/Temperature` and `sensor_msgs/FluidPressure` published by ZED Mini and ZED 2.

For a complete explanation of the tutorials please refer to the Stereolabs ZED online documentation:

- [Video](https://docs.stereolabs.com/integrations/ros/video/)
- [Depth](https://docs.stereolabs.com/integrations/ros/depth_sensing/)
- [Positional Tracking](https://docs.stereolabs.com/integrations/ros/positional_tracking/)``
- [Object Detection](https://docs.stereolabs.com/integrations/ros/object-detection/)
- [Sensors Data](https://docs.stereolabs.com/integrations/ros/sensors-data/)
