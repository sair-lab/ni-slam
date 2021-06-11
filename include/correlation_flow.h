#ifndef CORRELATION_FLOW_H
#define CORRELATION_FLOW_H

#include <fftw3.h>
#include "read_configs.h"

class CorrelationFlow{
public:
  CorrelationFlow(CFConfig& cf_config);
  void FFT(Eigen::ArrayXXf& image, Eigen::ArrayXXcf& fft_result);
  void ComputePose(Eigen::ArrayXXcf& last_fft_result, Eigen::ArrayXXcf& fft_result, Eigen::Vector3d& pose);
};

#endif  // CORRELATION_FLOW_H