/**
 * @file ocam_image_undistort_node.cpp
 * @author ZXW2600 (zhaoxinwei74@gmail.com)
 * @brief   采用 OCam 模型进行广角相机去畸变,并发布新的图像和相机参数
 * @version 0.1
 * @date 2021-12-28
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include "camera_model.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <thread>

boost::shared_ptr<image_transport::ImageTransport> it_;
image_transport::CameraSubscriber sub_camera_;
int queue_size_;

boost::mutex connect_mutex_;
image_transport::CameraPublisher pub_rect_;
image_transport::CameraPublisher h_rect_;
image_transport::CameraPublisher s_rect_;
image_transport::CameraPublisher v_rect_;
// Shared parameters to be propagated to nodelet private namespaces
boost::shared_ptr<ros::NodeHandle> private_nh;

camera_model _model("/home/zxw2600/workspace/ws_icra/robot_ws/src/"
                    "ocam_image_undistort/config/MER139-omni.xml");

void imageCb(const sensor_msgs::ImageConstPtr &image_msg,
             const sensor_msgs::CameraInfoConstPtr &info_msg) {
  // Verify camera is actually calibrated
  if (info_msg->K[0] == 0.0) {
    ROS_ERROR("Rectified topic '%s' requested but camera publishing '%s' "
              "is uncalibrated",
              pub_rect_.getTopic().c_str(), sub_camera_.getInfoTopic().c_str());
    return;
  }

  // If zero distortion, just pass the message along
  bool zero_distortion = true;
  for (size_t i = 0; i < info_msg->D.size(); ++i) {
    if (info_msg->D[i] != 0.0) {
      zero_distortion = false;
      break;
    }
  }
  // This will be true if D is empty/zero sized
  if (zero_distortion) {
    pub_rect_.publish(*image_msg.get(), _model.getCameraInfo());
    return;
  }

  // Create cv::Mat views onto both buffers
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat rect;
  rect = _model.undistortImage(image);
  cv::Mat image_hsv;
  cv::cvtColor(rect, image_hsv, cv::COLOR_BGR2HSV);
  //使用Mat容器，用at访问
  cv::Mat channels[3];
  split(image_hsv, channels);
  // Allocate new rectified image message
  sensor_msgs::ImagePtr rect_msg =
      cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect)
          .toImageMsg();
  // Allocate new rectified image message
  sensor_msgs::ImagePtr h_msg =
      cv_bridge::CvImage(image_msg->header, image_msg->encoding, channels[0])
          .toImageMsg(); // Allocate new rectified image message
  sensor_msgs::ImagePtr s_msg =
      cv_bridge::CvImage(image_msg->header, image_msg->encoding, channels[1])
          .toImageMsg(); // Allocate new rectified image message
  sensor_msgs::ImagePtr v_msg =
      cv_bridge::CvImage(image_msg->header, image_msg->encoding, channels[2])
          .toImageMsg();

  ros::Time now = ros::Time::now();
  auto info = _model.getCameraInfo();
  rect_msg->header.stamp = now;
  h_msg->header.stamp = now;
  s_msg->header.stamp = now;
  v_msg->header.stamp = now;
  info.header.stamp = now;
  pub_rect_.publish(*rect_msg.get(), info);
  h_rect_.publish(*h_msg.get(), info);
  s_rect_.publish(*s_msg.get(), info);
  v_rect_.publish(*v_msg.get(), info);
}

// Handles (un)subscribing when clients (un)subscribe
void connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_rect_.getNumSubscribers() == 0)
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints());
    sub_camera_ =
        it_->subscribeCamera("image_raw", queue_size_, imageCb, hints);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ocam_image_undistort");
  private_nh.reset(new ros::NodeHandle(""));
  // Check for common user errors
  if (ros::names::remap("camera") != "camera") {
    ROS_WARN("Remapping 'camera' has no effect! Start image_proc in the "
             "camera namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun image_proc image_proc",
             ros::names::remap("camera").c_str());
  }
  if (ros::this_node::getNamespace() == "/") {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start "
             "image_proc "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun image_proc image_proc");
  }

  it_.reset(new image_transport::ImageTransport(*private_nh.get()));
  // Read parameters
  private_nh->param("queue_size", queue_size_, 5);

  _model.getUndistrotMaps();
  // // Set up dynamic reconfigure
  // reconfigure_server_.reset(new ReconfigureServer(config_mutex_,
  // private_nh)); ReconfigureServer::CallbackType f =
  //     boost::bind(&RectifyNodelet::configCb, this, _1, _2);
  // reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&connectCb);
  { // Make sure we don't enter connectCb() between advertising and assigning to
    // pub_rect_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_rect_ =
        it_->advertiseCamera("rect/image_raw", 1, connect_cb, connect_cb);
    h_rect_ = it_->advertiseCamera("h/image_raw", 1, connect_cb, connect_cb);
    s_rect_ = it_->advertiseCamera("s/image_raw", 1, connect_cb, connect_cb);
    v_rect_ = it_->advertiseCamera("v/image_raw", 1, connect_cb, connect_cb);
  }
  ros::spin();
  return 0;
}
