#ifndef CAMERA_H_
#define CAMERA_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

class Camera{
public:
  Camera();
  Camera(const std::string& camera_file);
  Camera& operator=(const Camera& camera); // deep copy
  
  void UndistortImage(cv::Mat& image, cv::Mat& undistort_image);
  void GetNewCameraMatrix(cv::Mat& camera_matrix);

private:
  int _image_height;
  int _image_width;
  double _height;
  cv::Mat _K;
  cv::Mat _new_K;
  cv::Mat _D;
  cv::Mat _map1;
  cv::Mat _map2;
};

#endif  // CAMERA_H_