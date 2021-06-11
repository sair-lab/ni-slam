
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
  FramePtr last_frame;
  int edge_id = 0;
  size_t dataset_length = dataset.GetDatasetLength();
  const bool OdomPoseIsAvailable = dataset.PoseIsAvailable();
  for(size_t i = 0; i < dataset_length; ++i){
    // 1. read image and undistort
    cv::Mat image, undistort_image;
    if(!dataset.GetImage(image, i)){
      std::cout << "can not get image " << i << std::endl;
      break;
    }
    camera.UndistortImage(image, undistort_image);

    // 2. compute fft result
    Eigen::ArrayXXf image_array;
    ConvertMatToNormalizedArray(image, image_array);

    Eigen::ArrayXXcf fft_result = correlation_flow.FFT(image_array);

    // 3. construct frame
    FramePtr frame(new Frame(i, fft_result));

    // 4. initialize
    if(!init){
      Eigen::Vector3d pose;
      if(OdomPoseIsAvailable){
        dataset.GetPose(pose, i);
      }else{
        pose << 0.0, 0.0, 0.0;
      }
      frame->SetPose(pose);
      last_frame = frame;
      init = true;
      continue;
    }

    // 5. tracking, compute relative pose
    Eigen::Vector3d relative_pose;
    Eigen::ArrayXXcf last_fft_result;
    last_frame->GetFFTResult(last_fft_result);
    correlation_flow.ComputePose(last_fft_result, fft_result, relative_pose);

    // 6. set pose of current frame
    Eigen::Vector3d pose;
    if(OdomPoseIsAvailable){
      dataset.GetPose(pose, i);
    }else{
      Eigen::Vector3d last_pose;
      last_frame->GetPose(last_pose);
      pose = last_pose + relative_pose;
    }
    frame->SetPose(pose);

    // 7. add edges between last frame and currect frame to map
    int last_frame_id = last_frame->GetFrameId();
    int frame_id = frame->GetFrameId();

    EdgePtr cf_edge = std::make_shared<Edge>();
    cf_edge->_edge_id = edge_id++;
    cf_edge->_type = Edge::Type::KCC;
    cf_edge->_from = last_frame_id;
    cf_edge->_to = frame_id;
    cf_edge->_T = relative_pose;
    cf_edge->_information = Eigen::Matrix3d::Identity();
    map->AddEdge(cf_edge);

    if(OdomPoseIsAvailable){
      Eigen::Vector3d last_odom_pose, odom_pose, relative_odom_pose;
      dataset.GetPose(last_odom_pose, (i-1));
      dataset.GetPose(odom_pose, i);
      relative_odom_pose = odom_pose - last_odom_pose;

      EdgePtr odom_edge = std::make_shared<Edge>();
      odom_edge->_edge_id = edge_id++;
      odom_edge->_type = Edge::Type::Odom;
      odom_edge->_from = last_frame_id;
      odom_edge->_to = frame_id;
      odom_edge->_T = relative_odom_pose;
      odom_edge->_information = Eigen::Matrix3d::Identity();
      map->AddEdge(cf_edge);
    }

    // find loop closure
    


    // n. update last_frame
    last_frame = frame;
  }



};
