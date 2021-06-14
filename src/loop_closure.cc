#include "loop_closure.h"

LoopClosure::LoopClosure(double position_thr, double angle_thr, double response_thr, MapPtr map):
  _position_thr(position_thr), _angle_thr(angle_thr), _response_thr(response_thr), _map(map){
}

bool LoopClosure::FindLoopClosure(FramePtr& current_frame, FramePtr& loop_frame){
  

}

bool LoopClosure::FindLoopClosure(FramePtr& current_frame, FramePtr& loop_frame, Eigen::Vector3d& prior_pose);