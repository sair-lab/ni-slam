#ifndef MAP_STITCHER_H_
#define MAP_STITCHER_H_

#include "optimization_2d/pose_graph_2d_error_term.h"
#include "camera.h"
#include "frame.h"
#include "map.h"

typedef std::unordered_map<GridLocation, std::vector<int>, GridLocationHash, GridLocationEqual> OccupancyData;

class MapStitcher{
public:
  MapStitcher(CameraPtr camera);
  void InsertFrame(FramePtr frame, cv::Mat& image);
  void AddImageToOccupancy(FramePtr frame);
  void RecomputeOccupancy();
  OccupancyData& GetOccupancyData();

private:
  CameraPtr _camera;
  std::unordered_map<FramePtr, Eigen::MatrixXi> _raw_images;
  OccupancyData _occupancy_data;
};

typedef std::shared_ptr<MapStitcher> MapStitcherPtr;

#endif  // MAP_STITCHER_H_