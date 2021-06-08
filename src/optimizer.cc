#include <set>
#include <string>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "optimizer.h"
#include "frame.h"
#include "edge.h"
#include "map.h"
#include "optimization_2d/types.h"
#include "optimization_2d/pose_graph_2d.h"


Optimizer::Optimizer(std::shared_ptr<Map> map): _map(map){
}

void Optimizer::OptimizeMap(){
  std::map<int, ceres::optimization_2d::Pose2d> poses;
  std::vector<ceres::optimization_2d::Constraint2d> constraints;

  std::vector<Frame> frames;
  std::set<int> frame_ids;
  std::vector<Edge> edges;
  _map->GetAllFrames(frames);
  _map->GetAllEdges(edges);

  for(Frame& frame : frames){
    int frame_id = frame.GetFrameId();
    Eigen::Vector3d pose_vec;
    frame.GetPose(pose_vec);

    frame_ids.insert(frame_id);
    ceres::optimization_2d::Pose2d pose_2d;
    pose_2d.x = pose_vec(0);
    pose_2d.y = pose_vec(1);
    pose_2d.yaw_radians = pose_vec(2);
    poses.insert(std::pair<int, ceres::optimization_2d::Pose2d>(frame_id, pose_2d));
  }

  for(Edge& edge : edges){
    int from = edge._from;
    int to = edge._to;
    if((frame_ids.count(from) < 1) || (frame_ids.count(to) < 1)){
      continue;
    }

    ceres::optimization_2d::Constraint2d constraint;
    constraint.id_begin = from;
    constraint.id_end = to;
    constraint.x = edge._T(0);
    constraint.y = edge._T(1);
    constraint.yaw_radians = edge._T(2);
  }

  ceres::Problem problem;
  ceres::optimization_2d::BuildOptimizationProblem(constraints, &poses, &problem);
  CHECK(ceres::optimization_2d::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

  
}