#include "camera_model.hpp"
#include <algorithm>
#include <iostream>
#include <stdint.h>
using namespace cv;
using namespace std;

void camera_model::readCamParamFromFile(string cameraParamterFilePath) {
  //读取相机参数
  // read a xml file
  cv::FileStorage fs_read(cameraParamterFilePath, cv::FileStorage::READ);
  std::cout << "Reading " << cameraParamterFilePath << std::endl;
  if (!fs_read.isOpened()) {
    printf("camera Paramter File opened Failed!");
  } else {
    fs_read["image_width"] >> image_width;
    fs_read["image_height"] >> image_height;
    fs_read["camera_matrix"] >> camera_matrix;
    fs_read["camera_matrix_new"] >> camera_matrix_new;
    fs_read["distortion_coefficients"] >> dist_coeffs;
    fs_read["xi"] >> xi;

    int model;
    fs_read["model"] >> model;

    std::cout << "camera parms: height " << image_height << " width "
              << image_width << "model " << model << std::endl;
    camera_model_type = (Model)(model);
  }

  fs_read.release();
}

Mat camera_model::undistortImage(Mat image_ori) {
  if (undistort_map_inited) {
    Mat image_undistro;
    remap(image_ori, image_undistro, map_x, map_y, INTER_LINEAR);
    return image_undistro;
  } else {
    std::cout << "undistort_map not inited" << std::endl;
    return image_ori;
  }
}

void camera_model::getUndistrotMaps() {

  switch (camera_model_type) {
  case Model::NORMAL:
    // 0 for normal camera model
    camera_matrix.inv(0);
    initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat(),
                            camera_matrix_new, Size(image_width, image_height),
                            CV_32FC1, map_x, map_y);
    break;

  case Model::FISHEYE:
    // 1 for fisheye model
    fisheye::initUndistortRectifyMap(
        camera_matrix, dist_coeffs, Mat::eye(3, 3, 6), camera_matrix_new,
        Size(undist_image_width, undist_image_height), CV_32FC1, map_x, map_y);
    break;

  case Model::OMNICAM:
    // 2 for omnidirtional model
    omnidir::initUndistortRectifyMap(
        camera_matrix, dist_coeffs, xi, Mat::eye(3, 3, CV_64F),
        camera_matrix_new, Size(image_width, image_height), CV_16SC2, map_x,
        map_y, omnidir::RECTIFY_PERSPECTIVE);
    break;
  default:
    break;
  }

  // check if got right map
  if (map_x.empty() || map_y.empty()) {
    // error("camera undistort map get Failed! undistort process desabled!");
    undistort_map_inited = false;
  } else {
    undistort_map_inited = true;
  }

  undistort_map_inited = true;
}

camera_model::camera_model(std::string config_file_name) {
  initFromFile(config_file_name);
}

bool camera_model::initFromFile(std::string config_file_name) {
  readCamParamFromFile(config_file_name);
  if (!camera_matrix.empty()) {
    printf("Loading camera matrix successed\n");
    getUndistrotMaps();
    return true;
  }
  return false;
}

camera_model::camera_model() {}

sensor_msgs::CameraInfo
camera_model::getCameraInfo(void) { // extract cameraInfo.
  sensor_msgs::CameraInfo cam;

  cv::Mat k = camera_matrix_new;
  std::vector<double> D{0, 0, 0, 0, 0};
  boost::array<double, 9> K = {
      k.at<double>(0), k.at<double>(1), k.at<double>(2),
      k.at<double>(3), k.at<double>(4), k.at<double>(5),
      k.at<double>(6), k.at<double>(7), k.at<double>(8),
  };

  // cv::Mat p = camera_matrix_new; // get rectified projection.
  // boost::array<double, 12> P = {
  //     p.at<double>(0, 0), p.at<double>(0, 1), p.at<double>(0, 2),
  //     p.at<double>(0, 3), p.at<double>(1, 0), p.at<double>(1, 1),
  //     p.at<double>(1, 2), p.at<double>(1, 3), p.at<double>(2, 0),
  //     p.at<double>(2, 1), p.at<double>(2, 2), p.at<double>(2, 3)};
  // boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  cam.height = image_height;
  cam.width = image_width;
  cam.distortion_model = "plumb_bob";
  cam.D = D;
  cam.K = K;
  cam.binning_x = 0;
  cam.binning_y = 0;
  return cam;
}
