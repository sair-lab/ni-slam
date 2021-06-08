
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>

#include "dataset.h"
#include "edge.h"

int main(int argc, char** argv){
  google::InitGoogleLogging(argv[0]);

  std::string dataroot = "/media/xukuan/0000678400004823/dataset/ground_texture/uniform/1220";
  Dataset dataset(dataroot);

  size_t dataset_length = dataset.GetDatasetLength();
  for(size_t i = 0; i < dataset_length; ++i){
    cv::Mat image;
    if(dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }

    

  }



};
