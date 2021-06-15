#include "loop_closure.h"

LoopClosure::LoopClosure(double response_thr, CorrelationFlowPtr correlation_flow, MapPtr map): 
    _response_thr(response_thr), _correlation_flow(correlation_flow), _map(map){
}

LoopClosureResult LoopClosure::FindLoopClosure(FramePtr& current_frame){
  std::vector<FramePtr> frames;
  _map->GetAllFrames(frames);
  LoopClosureResult result = FindLoopClosure(current_frame, frames);
  return result;
}

LoopClosureResult LoopClosure::FindLoopClosure(FramePtr& current_frame, Eigen::Vector3d& prior_pose){
  std::vector<GridLocation> grid_locations;
  GridLocation grid_location = _map->ComputeGridLocation(prior_pose);
  for(int i = -1; i <= 1; i++){
    for(int j = -1; j <= 1; j++){
      GridLocation gl = grid_location;
      gl.x += i;
      gl.y += j;
      grid_locations.emplace_back(gl);
    }
  }

  std::vector<FramePtr> frames; 
  _map->GetFramesInGrids(frames, grid_locations);
  LoopClosureResult result = FindLoopClosure(current_frame, frames);
  return result;
}

LoopClosureResult LoopClosure::FindLoopClosure(FramePtr& current_frame, std::vector<FramePtr>& frames){
  Eigen::ArrayXXcf current_fft_result, current_fft_polar;
  current_frame->GetFFTResult(current_fft_result, current_fft_polar);

  LoopClosureResult result;
  result.current_frame = current_frame;
  for(FramePtr frame : frames){
    Eigen::ArrayXXcf fft_result, fft_polar;
    frame->GetFFTResult(fft_result, fft_polar);
    Eigen::Vector3d relative_pose;
    Eigen::Vector3d var = _correlation_flow->ComputePose(fft_result, current_fft_result, fft_polar, current_fft_polar, relative_pose);
    if(var.sum() > result.response){
      result.response = var.sum();
      result.loop_frame = frame;
      result.relative_pose = relative_pose;
    }
  }

  result.found = (result.response > _response_thr);
  return result;
}