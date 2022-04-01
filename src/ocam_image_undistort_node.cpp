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

// Shared parameters to be propagated to nodelet private namespaces
boost::shared_ptr<ros::NodeHandle> ptr_nh;

camera_model _model;

std::string sub_topic;
std::string pub_camera;
std::string config_file;

void imageCb(const sensor_msgs::ImageConstPtr &image_msg,
             const sensor_msgs::CameraInfoConstPtr &info_msg) {

  // Create cv::Mat views onto both buffers
  const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;

  cv::Mat rect;
  rect = _model.undistortImage(image);

  // Allocate new rectified image message
  sensor_msgs::ImagePtr rect_msg =
      cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect)
          .toImageMsg();

  ros::Time now = ros::Time::now();

  auto info = _model.getCameraInfo();
  rect_msg->header.stamp = now;

  info.header.stamp = now;
  pub_rect_.publish(*rect_msg.get(), info);
}

// Handles (un)subscribing when clients (un)subscribe
void connectCb() {
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_rect_.getNumSubscribers() == 0)
    sub_camera_.shutdown();
  else if (!sub_camera_) {
    image_transport::TransportHints hints("raw", ros::TransportHints());
    sub_camera_ = it_->subscribeCamera(sub_topic, queue_size_, imageCb, hints);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ocam_image_undistort");
  ptr_nh.reset(new ros::NodeHandle("~"));
  ptr_nh->setParam("testparam", "test");


  if (!ptr_nh->hasParam("subtopic")) {
    ROS_ERROR("subtopic paramter needed!");
    // return -1;
  } else {
    ptr_nh->getParam("subtopic", sub_topic);
  }
  if (!ptr_nh->hasParam("pubcamera")) {
    ROS_ERROR("pubcamera paramter needed!");
    // return -1;
  } else
    ptr_nh->getParam("pubcamera", pub_camera);

  if (!ptr_nh->hasParam("configfile")) {
    ROS_ERROR("configfile paramter needed!");
    // return -1;
  }
  else
    ptr_nh->getParam("configfile", config_file);


  ROS_INFO("Subscribe to camera : %s", sub_topic.c_str());
  ROS_INFO("Publish new image to camera : %s", pub_camera.c_str());
  ROS_INFO("Camera Paramter Config File : %s", config_file.c_str());


  it_.reset(new image_transport::ImageTransport(*ptr_nh.get()));

  // Read parameters
  ptr_nh->param("queue_size", queue_size_, 5);

  // init
  _model.initFromFile(config_file); 
  _model.getUndistrotMaps();

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb =
      boost::bind(&connectCb);
  {
    // Make sure we don't enter connectCb() between advertising and assigning to
    // pub_rect_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_rect_ = it_->advertiseCamera(pub_camera, 1, connect_cb, connect_cb);
  }

  ros::spin();
  return 0;
}
