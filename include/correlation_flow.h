#ifndef CORRELATION_FLOW_H
#define CORRELATION_FLOW_H

#include <fftw3.h>
#include "read_configs.h"

class CorrelationFlow{
public:
  CorrelationFlow(CFConfig& cf_config);
  Eigen::ArrayXXcf FFT(Eigen::ArrayXXf&);
  Eigen::ArrayXXf IFFT(Eigen::ArrayXXcf&);
  void ComputePose(Eigen::ArrayXXcf&, Eigen::ArrayXXcf&, Eigen::Vector3d&);
};

#endif  // CORRELATION_FLOW_H