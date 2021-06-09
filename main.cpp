
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

int main(int argc, char** argv){
  google::InitGoogleLogging(argv[0]);

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config;
  Dataset dataset(dataset_config.dataroot);
  Camera camera(dataset_config.camera_file);

  CFConfig cf_config = configs.cf_config;
  CorrelationFlow correlation_flow;

  std::shared_ptr<Map> map = std::make_shared<Map>();
  Optimizer optimizer(map);

  size_t dataset_length = dataset.GetDatasetLength();
  for(size_t i = 0; i < dataset_length; ++i){
    // std::cout << "i = " << i << std::endl;
    cv::Mat image, undistort_image;
    if(!dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }
    camera.UndistortImage(image, undistort_image);
    
    

  }



};
