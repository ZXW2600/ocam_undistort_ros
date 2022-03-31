/**
 * @file camera_model.hpp
 * @author ZXW2600 (zhaoxinwei74@gmail.com)
 * @brief 完成相机参数解析,图片去畸变
 * @version 0.1
 * @date 2021-12-12
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef __CAMERA_MODEL_HPP__
#define __CAMERA_MODEL_HPP__

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/opencv.hpp>

class camera_model {
public:
  cv::Mat camera_matrix, camera_matrix_new, dist_coeffs, map_x, map_y;
  double xi;

  bool undistort_map_inited = false;
  int image_width, image_height, camera_index;

  int undist_image_width, undist_image_height;

  enum class Model {
    NORMAL,  // 0 for normal camera model
    FISHEYE, // 1 for fisheye model
    OMNICAM  // 2 for omnidirtional model
  } camera_model_type;

  camera_model(std::string config_file_name);
  camera_model();

  bool initFromFile(std::string config_file_name);
  // used for undistort
  void getUndistrotMaps();
  /**
   * @brief 从标定文件读出相机参数
   *
   * @param cameraParamterFilePath 标定文件的相对位置
   */
  void readCamParamFromFile(std::string cameraParamterFilePath);

  sensor_msgs::CameraInfo getCameraInfo(void);
  cv::Mat undistortImage(cv::Mat image);
};

#endif // __CAMERA_MODEL_HPP__
