#ifndef MAP_STITCHER_H_
#define MAP_STITCHER_H_

#include "optimization_2d/pose_graph_2d_error_term.h"
#include "frame.h"
#include "map.h"

class MapStitcher{
public:
  MapStitcher();
  void InsertFrame(FramePtr frame, cv::Mat& image);
  void AddImageToOccupancy(FramePtr frame);
  void RecomputeOccupancy();

private:
  std::unordered_map<FramePtr, Eigen::MatrixXi> _raw_images;
  std::unordered_map<GridLocation, std::vector<int>, GridLocationHash, GridLocationEqual> _occupancy_data;
};

#endif  // MAP_STITCHER_H_