
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

#include "read_configs.h"
#include "dataset.h"
#include "camera.h"
#include "correlation_flow.h"
#include "map.h"
#include "optimizer.h"

int main(int argc, char** argv){
  google::InitGoogleLogging(argv[0]);

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config;
  Dataset dataset(dataset_config.dataroot);
  Camera camera(dataset_config.camera_file);

  CFConfig cf_config = configs.cf_config;
  CorrelationFlow correlation_flow(cf_config);

  std::shared_ptr<Map> map = std::make_shared<Map>();
  Optimizer optimizer(map);

  bool init = false;
  Frame last_frame;
  size_t dataset_length = dataset.GetDatasetLength();
  for(size_t i = 0; i < dataset_length; ++i){
    // std::cout << "i = " << i << std::endl;
    cv::Mat image, undistort_image;
    if(!dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }
    Eigen::Vector3d pose;
    dataset.GetPose(pose, i);
    camera.UndistortImage(image, undistort_image);

    // Eigen::ArrayXXf image_array = Eigen::Map<Eigen::ArrayXXf>(
    //     &undistort_image.at<float>(0,0), undistort_image.cols, undistort_image.rows);
    Eigen::ArrayXXf image_array;
    ConvertMatToArray(image, image_array);

    Eigen::ArrayXXf fft_result;
    correlation_flow.FFT(image_array, fft_result);
    Frame frame(i, fft_result, pose);
    if(!init){
      last_frame = frame;
      init = true;
      continue;
    }
    Eigen::Vector3d cf_pose;
    Eigen::ArrayXXf last_fft_result;
    last_frame.GetFFTResult(last_fft_result);
    correlation_flow.ComputePose(last_fft_result, fft_result, cf_pose);

    last_frame = frame;
  }



};
