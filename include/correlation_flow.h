#ifndef CORRELATION_FLOW_H
#define CORRELATION_FLOW_H

#include "read_configs.h"

class CorrelationFlow{
public:
  CorrelationFlow(CFConfig& cf_config);
  void FFT(Eigen::ArrayXXf& image, Eigen::ArrayXXf& fft_result);
  void ComputePose(Eigen::ArrayXXf& last_fft_result, Eigen::ArrayXXf& fft_result, Eigen::Vector3d& pose);
};

#endif  // CORRELATION_FLOW_H