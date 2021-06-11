#include "correlation_flow.h"
#include "read_configs.h"

CorrelationFlow::CorrelationFlow(CFConfig& cf_config){
  
}

void CorrelationFlow::FFT(Eigen::ArrayXXf& x, Eigen::ArrayXXcf& xf){

    xf = Eigen::ArrayXXcf(x.rows()/2+1, x.cols());

    fftwf_plan fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()),
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major

    fftwf_execute(fft_plan);

}

void CorrelationFlow::ComputePose(Eigen::ArrayXXcf& last_fft_result,
    Eigen::ArrayXXcf& fft_result, Eigen::Vector3d& pose){

}