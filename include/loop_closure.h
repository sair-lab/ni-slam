#ifndef LOOP_CLOSURE_H_
#define LOOP_CLOSURE_H_

#include "correlation_flow.h"
#include "map.h"

struct LoopClosureResult{
  bool found;
  float response;
  FramePtr current_frame;
  FramePtr loop_frame;
  Eigen::Vector3d relative_pose;

  LoopClosureResult(): found(false), response(-1) {}
  LoopClosureResult(bool _found, float _resopnse): found(_found), response(_resopnse) {}
  LoopClosureResult& operator=(const LoopClosureResult& other){
    found = other.found;
    response = other.response;
    current_frame = other.current_frame;
    loop_frame = other.loop_frame;
    relative_pose = other.relative_pose;
    return *this;
  }
};

class LoopClosure{
public:
  LoopClosure(double response_thr, CorrelationFlowPtr correlation_flow, MapPtr map);
  LoopClosureResult FindLoopClosure(FramePtr& current_frame);
  LoopClosureResult FindLoopClosure(FramePtr& current_frame, Eigen::Vector3d& prior_pose);
  LoopClosureResult FindLoopClosure(FramePtr& current_frame, std::vector<FramePtr>& frames);

private:
  double _response_thr;
  CorrelationFlowPtr _correlation_flow;
  MapPtr _map;
};

#endif  // LOOP_CLOSURE_H_