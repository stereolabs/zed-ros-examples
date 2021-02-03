///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/**
 * This tutorial demonstrates simple receipt of ZED depth messages over the ROS system.
 */

#include <ros/ros.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

/**
 * Subscriber callback
 */

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Accel: %.3f,%.3f,%.3f [m/s^2] - Ang. vel: %.3f,%.3f,%.3f [deg/sec] - Orient. Quat: %.3f,%.3f,%.3f,%.3f",
           msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x,
           msg->angular_velocity.y, msg->angular_velocity.z, msg->orientation.x, msg->orientation.y, msg->orientation.z,
           msg->orientation.w);
}

void imuTempCallback(const sensor_msgs::Temperature::ConstPtr& msg)
{
  ROS_INFO("IMU temperature: %.2f [C]", msg->temperature);
}

void leftTempCallback(const sensor_msgs::Temperature::ConstPtr& msg)
{
  ROS_INFO("Left CMOS temperature: %.2f [C]", msg->temperature);
}

void rightTempCallback(const sensor_msgs::Temperature::ConstPtr& msg)
{
  ROS_INFO("Right CMOS temperature: %.2f [C]", msg->temperature);
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
  ROS_INFO("Mag. Field: %.3f,%.3f,%.3f [uT]", msg->magnetic_field.x * 1e-6, msg->magnetic_field.y * 1e-6,
           msg->magnetic_field.z * 1e-6);
}

void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
  ROS_INFO("Atmospheric Pressure: %.2f [hPa]", msg->fluid_pressure * 100.f);
}

/**
 * Node main function
 */
int main(int argc, char** argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "zed_sensors_subscriber");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called imageCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber subImu = n.subscribe("/zed/zed_node/imu/data", 10, imuCallback);
  ros::Subscriber subImuTemp = n.subscribe("/zed/zed_node/imu/temperature", 10, imuTempCallback);
  ros::Subscriber subLeftTemp = n.subscribe("/zed/zed_node/temperature/left", 10, leftTempCallback);
  ros::Subscriber subRightTemp = n.subscribe("/zed/zed_node/temperature/right", 10, rightTempCallback);
  ros::Subscriber subPress = n.subscribe("/zed/zed_node/atm_press", 10, pressureCallback);
  ros::Subscriber subMag = n.subscribe("/zed/zed_node/imu/mag", 10, magCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
