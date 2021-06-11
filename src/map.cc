#include <map>
#include <vector>

#include "map.h"

Map::Map(){
}

void Map::AddFrame(FramePtr& frame){
  int frame_id = frame->GetFrameId();
  _frames[frame_id] = frame;
}

void Map::AddEdge(EdgePtr& edge){
  int edge_id = edge->_edge_id;
  int from = edge->_from;
  int to = edge->_to;

  _edges[edge_id] = edge;

  if(_frames.count(from) > 0){
    _frames[from]->AddEdge(edge_id);
  }  

  if(_frames.count(to) > 0){
    _frames[to]->AddEdge(edge_id);
  }
}

void Map::GetAllFrames(std::vector<FramePtr>& frames){
  for(auto kv : _frames){
    frames.emplace_back(kv.second);
  }
}

void Map::GetAllEdges(std::vector<EdgePtr>& edges){
  for(auto kv : _edges){
    edges.emplace_back(kv.second);
  }
}

void Map::UpdatePoses(AlignedMap<int, Eigen::Vector3d> frame_poses){
  for(auto kv : frame_poses){
    if(_frames.count(kv.first) > 0){
      _frames[kv.first]->SetPose(kv.second);
    }
  }
}