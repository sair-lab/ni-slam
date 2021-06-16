
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
#include "map_builder.h"


int main(int argc, char** argv){
  google::InitGoogleLogging(argv[0]);

  std::string config_file = argv[1];
  Configs configs(config_file);

  DatasetConfig dataset_config = configs.dataset_config;
  Dataset dataset(dataset_config.dataroot);

  const bool OdomPoseIsAvailable = dataset.PoseIsAvailable();
  MapBuilder map_builder(configs, OdomPoseIsAvailable);

  size_t dataset_length = dataset.GetDatasetLength();
  for(size_t i = 0; i < dataset_length; ++i){
    std::cout << i << std::endl;
    // if (i%5 != 0){
    //   continue;
    // }
    cv::Mat image;
    if(!dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }

    Eigen::Vector3d pose;
    if(OdomPoseIsAvailable){
      dataset.GetPose(pose, i);
    }
    map_builder.AddNewInput(image, pose);
  }



};
