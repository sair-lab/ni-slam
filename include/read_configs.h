#ifndef READ_CONFIGS_H_
#define READ_CONFIGS_H_

#include <iostream>
#include <yaml-cpp/yaml.h>

#include "utils.h"

struct DatasetConfig{
  std::string dataroot;
  std::string camera_file;
};

struct CFConfig{
  int width;
  int height;
  float sigma;
  float lambda;
};

struct LoopClosureConfig{
  double position_threshold;
  double angle_threshold;
  double response_threshold;
}

struct Configs{
  DatasetConfig dataset_config;
  CFConfig cf_config;

  Configs(const std::string& config_file){
    if(!FileExists(config_file)){
      std::cout << "config file: " << config_file << " doesn't exist" << std::endl;
      return;
    }

    YAML::Node file_node = YAML::LoadFile(config_file);
    YAML::Node dataset_node = file_node["dataset"];
    dataset_config.dataroot = dataset_node["dataroot"].as<std::string>();
    dataset_config.camera_file = dataset_node["camera_config"].as<std::string>();

    YAML::Node cf_node = file_node["correlation_flow"];
    cf_config.width = cf_node["width"].as<int>();
    cf_config.height = cf_node["height"].as<int>();
    cf_config.sigma = cf_node["sigma"].as<float>();
    cf_config.lambda = cf_node["lambda"].as<float>();
  }
};



#endif  // READ_CONFIGS_H_