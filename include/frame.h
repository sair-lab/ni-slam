#ifndef FRAME_H_
#define FRAME_H_

#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

class Frame{
public:
  Frame();
  Frame(int frame_id);
  Frame(int frame_id, Eigen::MatrixXd& fft_result, Eigen::Vector3d& pose);

  void SetFrameId(int frame_id);
  int GetFrameId();
  void SetFFTResult(Eigen::MatrixXd& fft_result);
  void GetFFTResult(Eigen::MatrixXd& fft_result);
  void SetPose(Eigen::Vector3d& pose);
  void GetPose(Eigen::Vector3d& pose);
  void AddEdge(int edge_id);
  void GetEdgeIds(std::vector<int>& edge_ids);
  void SaveToDisk(const std::string root_dir);

private:
  int _frame_id;
  Eigen::MatrixXd _fft_result;
  Eigen::Vector3d _pose;
  std::vector<int> _edge_ids;
};


#endif  // FRAME_H_