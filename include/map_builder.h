#ifndef MAP_BUILDER_H_
#define MAP_BUILDER_H

#include "read_configs.h"
#include "camera.h"
#include "frame.h"
#include "correlation_flow.h"
#include "map.h"
#include "loop_closure.h"

class MapBuilder{
public:
  MapBuilder(Configs& configs, const bool odom_is_available);

  void AddNewInput(cv::Mat& image, Eigen::Vector3d& odom_pose);
  void ComputeFFTResult(cv::Mat& image);
  void ConstructFrame();
  bool Initialize();
  void UpdateIntermedium();
  bool Tracking(Eigen::Vector3d& relative_pose);
  void AddCFEdgeToMap(Eigen::Vector3d& relative_pose);
  void AddOdomEdgeToMap();
  bool FindLoopClosure();
  bool OptimizeMap();

private:
  const bool OdomPoseIsAvailable;
  bool _init;
  int _frame_id;
  int _edge_id;
  
  // tmp
  FramePtr _last_frame;
  FramePtr _current_frame;
  Eigen::Vector3d _last_odom_pose;
  Eigen::Vector3d _current_odom_pose;
  Eigen::Vector3d _last_pose;
  Eigen::Vector3d _current_pose;

  // Intermedium rsults
  Eigen::ArrayXXf _image_array;
  Eigen::ArrayXXcf _last_fft_result;
  Eigen::ArrayXXcf _last_fft_polar;
  Eigen::ArrayXXcf _fft_result;
  Eigen::ArrayXXcf _fft_polar;
  std::vector<LoopClosureResult> _loop_matches;

  CameraPtr _camera;
  CorrelationFlowPtr _correlation_flow;
  MapPtr _map;
  LoopClosurePtr _loop_closure;
  // OptimizerPtr _optimizer;
};


#endif  // MAP_BUILDER_H
