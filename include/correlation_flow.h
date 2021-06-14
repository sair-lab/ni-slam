#ifndef CORRELATION_FLOW_H
#define CORRELATION_FLOW_H

#include <fftw3.h>
#include "utils.h"
#include "read_configs.h"

class CorrelationFlow{

public:
    CorrelationFlow(CFConfig& cf_config);
    Eigen::ArrayXXcf FFT(Eigen::ArrayXXf&);
    void ComputePose(Eigen::ArrayXXcf&, Eigen::ArrayXXcf&, Eigen::Vector3d&);

private:
    CFConfig cfg;
    Eigen::ArrayXXcf target_fft;
    Eigen::ArrayXXcf filter_fft;
    Eigen::ArrayXXf IFFT(Eigen::ArrayXXcf&);
    inline Eigen::ArrayXXcf gaussian_kernel(const Eigen::ArrayXXcf&);
    inline Eigen::ArrayXXcf gaussian_kernel(const Eigen::ArrayXXcf&, const Eigen::ArrayXXcf&);
    inline Eigen::ArrayXXf polar(const Eigen::ArrayXXf&);
};

#endif  // CORRELATION_FLOW_H