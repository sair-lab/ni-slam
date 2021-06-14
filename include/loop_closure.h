#ifndef LOOP_CLOSURE_H_
#define LOOP_CLOSURE_H_

#include "map.h"

class LoopClosure{
public:
  LoopClosure(double position_thr, double angle_thr, double response_thr, MapPtr map);
  bool FindLoopClosure(FramePtr& current_frame, FramePtr& loop_frame);
  bool FindLoopClosure(FramePtr& current_frame, FramePtr& loop_frame, Eigen::Vector3d& prior_pose);

private:
  double _position_thr;
  double _angle_thr;
  double _response_thr;
  MapPtr _map;
}

#endif  // LOOP_CLOSURE_H_