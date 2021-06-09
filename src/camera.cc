#include <yaml-cpp/yaml.h>

#include "camera.h"
#include "utils.h"

Camera::Camera(){
}

Camera::Camera(const std::string& camera_file){
  if(!FileExists(camera_file)){
    std::cout << "camera file: " << camera_file << " doesn't exist" << std::endl;
    return;
  }
  
  YAML::Node file_node = YAML::LoadFile(camera_file);

  _image_width = file_node["image_size"][0].as<int>();
  _image_height = file_node["image_size"][1].as<int>();
  _height = file_node["height"].as<double>();

  YAML::Node K_node = file_node["intrinsics"]["data"];
  double fx = K_node[0].as<double>();
  double cx = K_node[1].as<double>();
  double fy = K_node[2].as<double>();
  double cy = K_node[3].as<double>();
  _K = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

  YAML::Node D_node = file_node["distortion"]["data"];
  double D_data[5];
  for(size_t i = 0; i < 5; i++){
    D_data[i] = D_node[i].as<double>();
  }
  _D = (cv::Mat_<double>(5, 1) << D_data[0], D_data[1], D_data[2], D_data[3], D_data[4]);

  cv::Size image_size(_image_width, _image_height);
  _new_K = getOptimalNewCameraMatrix(_K, _D, image_size, 0, image_size);
  initUndistortRectifyMap(_K, _D, cv::Mat(), _new_K, image_size, CV_16SC2, _map1, _map2);
}

Camera& Camera::operator=(const Camera& camera){
  _image_height = camera._image_height;
  _image_width = camera._image_width;
  _height = camera._height;
  _K = camera._K.clone();
  _new_K = camera._new_K;
  _D = camera._D.clone();
  _map1 = camera._map1.clone();
  _map2 = camera._map2.clone();
  return *this;
}

void Camera::UndistortImage(cv::Mat& image, cv::Mat& undistort_image){
  remap(image, undistort_image, _map1, _map2, CV_INTER_LINEAR);
}

void Camera::GetNewCameraMatrix(cv::Mat& camera_matrix){
  camera_matrix = _new_K.clone();
}