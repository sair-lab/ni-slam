#include "frame.h"

Frame::Frame(){
}

Frame::Frame(int frame_id): _frame_id(frame_id){
}

Frame::Frame(int frame_id, Eigen::ArrayXXf& fft_result):
    _frame_id(frame_id), _fft_result(fft_result){
}

Frame& Frame::operator=(const Frame& other){
  _frame_id = other._frame_id;
  _fft_result = other._fft_result;
  _pose = other._pose;
  _edge_ids = other._edge_ids;
  return *this;
}

void Frame::SetFrameId(int frame_id){
  _frame_id = frame_id;
}

int Frame::GetFrameId(){
  return _frame_id;
}

void Frame::SetFFTResult(Eigen::ArrayXXf& fft_result){
  _fft_result = fft_result;
}

void Frame::SetFFTResult(Eigen::ArrayXXf& fft_result, 
    Eigen::ArrayXXf& depth_fft_result){
  _fft_result = fft_result;
  _depth_fft_result = depth_fft_result;
}

void Frame::GetFFTResult(Eigen::ArrayXXf& fft_result){
  fft_result = _fft_result;
}

void Frame::GetFFTResult(Eigen::ArrayXXf& fft_result, 
    Eigen::ArrayXXf& depth_fft_result){
  fft_result = _fft_result;
  depth_fft_result = _depth_fft_result;
}

void Frame::SetPose(Eigen::Vector3d& pose){
  _pose = pose;
}
void Frame::GetPose(Eigen::Vector3d& pose){
  pose = _pose;
}

void Frame::AddEdge(int edge_id){
  _edge_ids.emplace_back(edge_id);
}

void Frame::GetEdgeIds(std::vector<int>& edge_ids){
  edge_ids = _edge_ids;
}

void Frame::SaveToDisk(const std::string root_dir){
  //TODO
}