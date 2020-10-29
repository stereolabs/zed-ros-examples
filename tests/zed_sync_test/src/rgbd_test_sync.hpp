///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
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

#ifndef RGBD_TEST_SYNC_HPP
#define RGBD_TEST_SYNC_HPP

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

namespace zed_test_nodelets {

class ZEDTestRGBDSync : public nodelet::Nodelet {

public:
    ZEDTestRGBDSync();
    virtual ~ZEDTestRGBDSync();

protected:
    /*! \brief Initialization function called by the Nodelet base class
     */
    virtual void onInit();

    /*! \brief Reads parameters from the param server
     */
    void readParameters();

    /*! \brief Callback for RGBD topics synchronization
     */
    void callbackRGBD(
            const sensor_msgs::ImageConstPtr& rgb,
            const sensor_msgs::ImageConstPtr& depth,
            const sensor_msgs::CameraInfoConstPtr& rgbCameraInfo,
            const sensor_msgs::CameraInfoConstPtr& depthCameraInfo );

private:
    // Node handlers
    ros::NodeHandle mNh;    // Node handler
    ros::NodeHandle mNhP;  // Private Node handler

    // Publishers
    image_transport::CameraPublisher mPubSync;

    // Subscribers
    image_transport::SubscriberFilter mSubRgbImage;
    image_transport::SubscriberFilter mSubDepthImage;
    message_filters::Subscriber<sensor_msgs::CameraInfo> mSubRgbCamInfo;
    message_filters::Subscriber<sensor_msgs::CameraInfo> mSubDepthCamInfo;

    // Approx sync policies
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproxRgbdSyncPolicy;
    message_filters::Synchronizer<ApproxRgbdSyncPolicy>* mApproxRgbdSync = nullptr;

    // Exact sync policies
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactRgbdSyncPolicy;
    message_filters::Synchronizer<ExactRgbdSyncPolicy>* mExactRgbdSync = nullptr;

    // Params
    std::string mZedNodeletName = "zed_node";
    bool mUseApproxSync = true;
    int mQueueSize = 50;
    bool mVerbose=false;
    bool mSaveFrames=false;
    std::string mSavePath="~/ros_zed_sync_test";
};

} // namespace zed_test_nodelets

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(zed_test_nodelets::ZEDTestRGBDSync, nodelet::Nodelet)

#endif // RGBD_TEST_SYNC_HPP
