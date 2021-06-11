#include "correlation_flow.h"
#include "read_configs.h"

CorrelationFlow::CorrelationFlow(CFConfig& cf_config){
  
}

Eigen::ArrayXXcf CorrelationFlow::FFT(Eigen::ArrayXXf& x){

    Eigen::ArrayXXcf xf = Eigen::ArrayXXcf(x.rows()/2+1, x.cols());

    fftwf_plan fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()),
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major

    fftwf_execute(fft_plan);

    return xf;
}

Eigen::ArrayXXf CorrelationFlow::IFFT(Eigen::ArrayXXcf& xf){

    Eigen::ArrayXXf x = Eigen::ArrayXXf((xf.rows()-1)*2, xf.cols());

    Eigen::ArrayXXcf cxf = xf;

    fftwf_plan fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);

    fftwf_execute(fft_plan);

    return x/x.size();
}


void CorrelationFlow::ComputePose(Eigen::ArrayXXcf& last_fft_result,
    Eigen::ArrayXXcf& fft_result, Eigen::Vector3d& pose){

}