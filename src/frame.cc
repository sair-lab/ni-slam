#include "frame.h"

Frame::Frame(){
}

Frame::Frame(int frame_id): _frame_id(frame_id){
}

Frame::Frame(int frame_id, Eigen::MatrixXd& fft_result, Eigen::Vector3d& pose):
    _frame_id(frame_id), _fft_result(fft_result), _pose(pose){
}

void Frame::SetFrameId(int frame_id){
  _frame_id = frame_id;
}

int Frame::GetFrameId(){
  return _frame_id;
}

void Frame::SetFFTResult(Eigen::MatrixXd& fft_result){
  _fft_result = fft_result;
}

void Frame::GetFFTResult(Eigen::MatrixXd& fft_result){
  fft_result = _fft_result;
}

void Frame::SetPose(Eigen::Vector3d& pose){
  _pose = pose;
}
void Frame::GetPose(Eigen::Vecyor3d& pose){
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