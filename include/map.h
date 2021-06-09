#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include <map>

#include "frame.h"
#include "edge.h"
#include "utils.h"

class Map{
public:
  Map();
  void AddFrame(Frame& frame);
  void AddEdge(Edge& edge);
  
  void GetAllFrames(std::vector<Frame>& frames);
  void GetAllEdges(std::vector<Edge>& edges);

  void UpdatePoses(AlignedMap<int, Eigen::Vector3d> frame_poses);
private:
  std::map<int, Frame> _frames;
  std::map<int, Edge> _edges;
};

#endif  // MAP_H_