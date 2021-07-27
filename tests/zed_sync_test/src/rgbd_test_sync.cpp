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

#include "rgbd_test_sync.hpp"

#ifndef NDEBUG
#include <ros/console.h>
#endif

#include <chrono>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>

#include <sensor_msgs/image_encodings.h>

namespace zed_test_nodelets {

ZEDTestRGBDSync::ZEDTestRGBDSync() {

}

ZEDTestRGBDSync::~ZEDTestRGBDSync() {
    if(mApproxRgbdSync)
        delete mApproxRgbdSync;

    if(mExactRgbdSync)
        delete mExactRgbdSync;
}

void ZEDTestRGBDSync::onInit() {
    // Node handlers
    mNh = getNodeHandle();
    mNhP = getPrivateNodeHandle();

#ifndef NDEBUG
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
#endif

    NODELET_INFO( "********** Starting nodelet '%s' **********",getName().c_str() );

    readParameters();

    // Image publishers
    image_transport::ImageTransport it_sync(mNhP);

    mPubSync = it_sync.advertiseCamera("sync_image",1);
    NODELET_INFO_STREAM("Advertised on topic " << mPubSync.getTopic());
    NODELET_INFO_STREAM("Advertised on topic " << mPubSync.getInfoTopic());

    if( mUseApproxSync ) {
        NODELET_INFO( "Using Approximate Time sync");
        NODELET_INFO("RGB + Depth Sync" );
        mApproxRgbdSync = new message_filters::Synchronizer<ApproxRgbdSyncPolicy>(ApproxRgbdSyncPolicy(mQueueSize),
                                                                                  mSubRgbImage, mSubDepthImage,
                                                                                  mSubRgbCamInfo, mSubDepthCamInfo);
        mApproxRgbdSync->registerCallback(boost::bind(&ZEDTestRGBDSync::callbackRGBD, this,
                                                      _1, _2, _3, _4));

    } else {
        NODELET_INFO( "Using Exact Time sync");
        NODELET_INFO("RGB + Depth Sync" );

        mExactRgbdSync = new message_filters::Synchronizer<ExactRgbdSyncPolicy>(ExactRgbdSyncPolicy(mQueueSize),
                                                                                mSubRgbImage, mSubDepthImage,
                                                                                mSubRgbCamInfo, mSubDepthCamInfo);
        mExactRgbdSync->registerCallback(boost::bind(&ZEDTestRGBDSync::callbackRGBD, this,
                                                     _1, _2, _3, _4));

    }

    // Create remappings
    ros::NodeHandle rgb_nh(mNh, mZedNodeletName+"/rgb");
    ros::NodeHandle depth_nh(mNh, mZedNodeletName+"/depth");
    ros::NodeHandle rgb_pnh(mNhP, mZedNodeletName+"/rgb");
    ros::NodeHandle depth_pnh(mNhP, mZedNodeletName+"/depth");

    image_transport::ImageTransport rgb_it(rgb_nh);
    image_transport::ImageTransport depth_it(depth_nh);

    image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
    image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

    mSubRgbImage.subscribe(rgb_it, rgb_nh.resolveName("image_rect_color"), 1, hintsRgb);
    mSubDepthImage.subscribe(depth_it, depth_nh.resolveName("depth_registered"), 1, hintsDepth);
    mSubRgbCamInfo.subscribe(rgb_nh, "camera_info", 1);
    mSubDepthCamInfo.subscribe(depth_nh, "camera_info", 1);

    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubRgbImage.getTopic().c_str());
    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubRgbCamInfo.getTopic().c_str());
    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubDepthImage.getTopic().c_str());
    NODELET_INFO_STREAM( " * Subscribed to topic: " << mSubDepthCamInfo.getTopic().c_str());
}

void ZEDTestRGBDSync::readParameters() {
    NODELET_INFO("*** PARAMETERS [%s]***",  getName().c_str());

    mNhP.getParam("zed_nodelet_name", mZedNodeletName);
    mNhP.getParam("approx_sync", mUseApproxSync);
    mNhP.getParam("queue_size", mQueueSize);
    mNhP.getParam("verbose", mVerbose);
    mNhP.getParam("save_frames", mSaveFrames);
    mNhP.getParam("save_path", mSavePath);
    if(mSaveFrames) {
        if(mSavePath.empty()) {
            mSavePath = "~/ros_zed_sync_test";
        }

        if(mSavePath.back()!='/') {
            mSavePath += "/";
        }

        if(mSavePath.front()=='~') {
            mSavePath.erase(0,1);

            std::string home = "/home/" + std::string(getenv("USER"));
            mSavePath = home+mSavePath;
        }

        // ----> Create folder if not existing
        std::ifstream f(mSavePath.c_str());

        if(!f.good()) {
            std::string cmd = "mkdir -p " + mSavePath;
            int res = std::system( cmd.c_str() );
            if(res!=0)
            {
                NODELET_ERROR_STREAM("Error creating save path: " << res );
            }
        }
        // <---- Create folder if not existing
    }

    NODELET_INFO(" * zed_nodelet_name -> %s", mZedNodeletName.c_str());
    NODELET_INFO(" * approx_sync -> %s", mUseApproxSync?"true":"false");
    NODELET_INFO(" * queue_size  -> %d", mQueueSize);
    NODELET_INFO(" * verbose  -> %s", mVerbose?"true":"false");
    NODELET_INFO(" * save_frames  -> %s", mSaveFrames?"true":"false");
    NODELET_INFO(" * save_path  -> %s", mSavePath.c_str());
}

void ZEDTestRGBDSync::callbackRGBD(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth,
                                   const sensor_msgs::CameraInfoConstPtr &rgbCameraInfo,
                                   const sensor_msgs::CameraInfoConstPtr &depthCameraInfo) {
    if(mVerbose) {
        // ----> Frequency calculation
        static std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

        double elapsed_usec = std::chrono::duration_cast<std::chrono::microseconds>(now - last_time).count();
        last_time = now;

        double freq = 1e6/elapsed_usec;
        NODELET_INFO( "Freq: %.2f", freq);
        // <---- Frequency calculation
    }

    cv::Mat rgb_img(rgb->height, rgb->width, CV_8UC4, (void*)(&rgb->data[0]) );
    cv::cvtColor(rgb_img,rgb_img,cv::COLOR_BGRA2BGR);
    cv::Mat depth_img(depth->height, depth->width, CV_32FC1, (void*)(&depth->data[0]) );

    // Create color overlay for visual inspection.
    cv::Mat depth_rescaled;
    depth_img.convertTo(depth_rescaled, CV_8U, 255./5.);
    cv::Mat depth_colorized;
    cv::applyColorMap(depth_rescaled, depth_colorized, cv::COLORMAP_JET);
    cv::Mat sync_image;

    if(mVerbose) {
        ros::Time rgb_ts = rgb->header.stamp;
        ros::Time depth_ts = rgb->header.stamp;

        NODELET_INFO("[%u.%u] RGB: %dx%dx%d", rgb_ts.sec,rgb_ts.nsec, rgb_img.cols, rgb_img.rows, rgb_img.channels());
        NODELET_INFO("[%u.%u] DEPTH: %dx%dx%d", depth_ts.sec,depth_ts.nsec, depth_colorized.cols, depth_colorized.rows, depth_colorized.channels());
    }

    cv::addWeighted(rgb_img, 0.5, depth_colorized, 0.5, 0.0, sync_image);

    //cv::imshow("Sync image", sync_image);
    //cv::waitKey(1);

    if(mSaveFrames) {
        static uint64_t count=0;
        std::stringstream ss_filename;
        ss_filename << mSavePath << "ros_sync_frame_" << std::setfill('0') << std::setw(6) << count++ << ".png";

        bool saved = cv::imwrite( ss_filename.str(), sync_image );

        if(!saved) {
            NODELET_WARN_STREAM( "Error saving image: " << ss_filename.str());
        }
    }

    uint32_t sub_count = mPubSync.getNumSubscribers();
    if(sub_count==0) {
        return;
    }

    sensor_msgs::ImagePtr img_msg = boost::make_shared<sensor_msgs::Image>();

    img_msg->header.stamp = rgb->header.stamp;
    img_msg->header.frame_id = rgb->header.frame_id;
    img_msg->width = sync_image.cols;
    img_msg->height = sync_image.rows;
    int num = 1; // for endianness detection
    img_msg->is_bigendian = !(*(char*)&num == 1);

    img_msg->step = sync_image.step;
    img_msg->encoding = sensor_msgs::image_encodings::BGR8;

    size_t size = img_msg->step * img_msg->height;
    img_msg->data.resize(size);
    memcpy( (void*)(&img_msg->data[0]), (void*)(&sync_image.data[0]), size);

    mPubSync.publish( img_msg, rgbCameraInfo );

}

} // namespace zed_test_nodelets
