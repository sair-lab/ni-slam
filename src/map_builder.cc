#include "map_builder.h"

MapBuilder::MapBuilder(Configs& configs, const bool odom_is_available):
    OdomPoseIsAvailable(odom_is_available), _init(false), _frame_id(0), _edge_id(0){
  _camera = std::shared_ptr<Camera>(new Camera(configs.dataset_config.camera_file));
  _correlation_flow = std::shared_ptr<CorrelationFlow>(new CorrelationFlow(configs.cf_config));
  _map = std::shared_ptr<Map>(new Map(configs.map_config));

  _loop_closure = std::shared_ptr<LoopClosure>(
      new LoopClosure(configs.loop_closure_config, _correlation_flow, _map));
  _optimizer = std::shared_ptr<Optimizer>(new Optimizer(_map));
}

void MapBuilder::AddNewInput(cv::Mat& image, Eigen::Vector3d& odom_pose){
  if(OdomPoseIsAvailable){
    _current_odom_pose = odom_pose;
  }else{
    _current_odom_pose << 0.0, 0.0, 0.0;
  }
  cv::Mat undistort_image;
  _camera->UndistortImage(image, undistort_image);

  ComputeFFTResult(undistort_image);
  ConstructFrame();
  
  if(!_init){
    Initialize();
    UpdateIntermedium();
    return;
  }
  
  Eigen::Vector3d relative_pose;
  bool good_tracking = Tracking(relative_pose);
  if((!good_tracking) && (!OdomPoseIsAvailable)){
    return;
  }else if(good_tracking){
    AddCFEdgeToMap(relative_pose);
  }
  AddOdomEdgeToMap();

  // update current pose
  if(OdomPoseIsAvailable){
    _current_pose = _current_odom_pose;
  }else{
    Eigen::Vector3d last_pose;
    _last_frame->GetPose(last_pose);
    _current_pose = ComputeAbsolutePose(last_pose, relative_pose);
  }
  _current_frame->SetPose(_current_pose);

  FindLoopClosure();
  if(_loop_matches.size() >= 3){
    _optimizer->OptimizeMap();
  }

  UpdateIntermedium();
}

void MapBuilder::ComputeFFTResult(cv::Mat& image){
  ConvertMatToNormalizedArray(image, _image_array);
  _correlation_flow->ComputeIntermedium(_image_array, _fft_result, _fft_polar);
}

void MapBuilder::ConstructFrame(){
  _current_frame = std::shared_ptr<Frame>(
      new Frame(_frame_id++, _image_array, _fft_result, _fft_polar));
}

bool MapBuilder::Initialize(){
  if(!OdomPoseIsAvailable){
    _current_pose << 0.0, 0.0, 0.0;
  }else{
    _current_pose = _current_odom_pose;
  }
  _current_frame->SetPose(_current_pose);
  _init = true;
  return true;
}

void MapBuilder::UpdateIntermedium(){
  _last_frame = _current_frame;
  _last_odom_pose = _current_odom_pose;
  _last_fft_result = _fft_result;
  _last_fft_polar = _fft_polar;
}

bool MapBuilder::Tracking(Eigen::Vector3d& relative_pose){
  _correlation_flow->ComputePose(
      _last_fft_result, _fft_result, _last_fft_polar, _fft_polar, relative_pose);
  return true;
}

void MapBuilder::AddCFEdgeToMap(Eigen::Vector3d& relative_pose){
  EdgePtr cf_edge = std::make_shared<Edge>();
  cf_edge->_edge_id = _edge_id++;
  cf_edge->_type = Edge::Type::KCC;
  cf_edge->_from = _last_frame->GetFrameId();
  cf_edge->_to = _current_frame->GetFrameId();
  cf_edge->_T = relative_pose;
  cf_edge->_information = Eigen::Matrix3d::Identity();
  _map->AddEdge(cf_edge);
}

void MapBuilder::AddOdomEdgeToMap(){
  if(!OdomPoseIsAvailable) return;

  Eigen::Vector3d relative_odom_pose = 
      ComputeRelativePose(_last_odom_pose, _current_odom_pose);

  EdgePtr odom_edge = std::make_shared<Edge>();
  odom_edge->_edge_id = _edge_id++;
  odom_edge->_type = Edge::Type::Odom;
  odom_edge->_from = _last_frame->GetFrameId();
  odom_edge->_to = _current_frame->GetFrameId();
  odom_edge->_T = relative_odom_pose;
  odom_edge->_information = Eigen::Matrix3d::Identity();
  _map->AddEdge(odom_edge);
}

bool MapBuilder::FindLoopClosure(){
  LoopClosureResult loop_closure_result = _loop_closure->FindLoopClosure(_current_frame, _current_pose);
  if(loop_closure_result.found){
    _loop_matches.emplace_back(loop_closure_result);
  }else{
    _loop_matches.clear();
  }
  return loop_closure_result.found;
}