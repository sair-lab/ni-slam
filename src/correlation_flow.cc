#include "correlation_flow.h"
#include "read_configs.h"

CorrelationFlow::CorrelationFlow(CFConfig& cf_config)
{
    cfg = cf_config;
    Eigen::ArrayXXf target = Eigen::ArrayXXf::Zero(cfg.width, cfg.height);
    target(cfg.width/2, cfg.height/2) = 1;
    target_fft = FFT(target);
    Eigen::ArrayXXf zero = Eigen::ArrayXXf::Zero(cfg.width, cfg.height);
    filter_fft = FFT(zero);
}

Eigen::ArrayXXcf CorrelationFlow::FFT(const Eigen::ArrayXXf& x)
{

    Eigen::ArrayXXcf xf = Eigen::ArrayXXcf(x.rows()/2+1, x.cols());
    fftwf_plan fft_plan = fftwf_plan_dft_r2c_2d(x.cols(), x.rows(), (float(*))(x.data()),
        (float(*)[2])(xf.data()), FFTW_ESTIMATE); // reverse order for column major
    fftwf_execute(fft_plan);
    return xf;
}

Eigen::ArrayXXf CorrelationFlow::IFFT(const Eigen::ArrayXXcf& xf)
{

    Eigen::ArrayXXf x = Eigen::ArrayXXf((xf.rows()-1)*2, xf.cols());
    Eigen::ArrayXXcf cxf = xf;
    fftwf_plan fft_plan = fftwf_plan_dft_c2r_2d(xf.cols(), (xf.rows()-1)*2, (float(*)[2])(cxf.data()),
        (float(*))(x.data()), FFTW_ESTIMATE);
    fftwf_execute(fft_plan);
    return x/x.size();
}

void CorrelationFlow::ComputeIntermedium(const Eigen::ArrayXXf& image, Eigen::ArrayXXcf& fft_result, Eigen::ArrayXXcf& fft_polar)
{
    fft_result = FFT(image);
    fft_polar = FFT(polar(image));
}

float CorrelationFlow::ComputePose(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXcf& fft_result,
                                   const Eigen::ArrayXXcf& last_fft_polar, const Eigen::ArrayXXcf& fft_polar,
                                   Eigen::Vector3d& pose)
{
    Eigen::Vector2d trans, rots;
    auto var_trans = EstimateTrans(last_fft_result, fft_result, trans);
    auto var_rots = EstimateTrans(last_fft_result, fft_polar, rots);
}

float CorrelationFlow::EstimateTrans(const Eigen::ArrayXXcf& last_fft_result, const Eigen::ArrayXXcf& fft_result, Eigen::Vector2d& trans)
{
    auto Kzz = gaussian_kernel(last_fft_result);
    auto Kxz = gaussian_kernel(fft_result, last_fft_result);
    auto H = target_fft/(Kzz + cfg.lambda);
    Eigen::ArrayXXcf G = H*Kxz;
    Eigen::ArrayXXf g = IFFT(G);
    Eigen::ArrayXXf::Index max_index[2];
    auto max_response = g.maxCoeff(&(max_index[0]), &(max_index[1]));
    trans[0] = -(max_index[0]-cfg.width/2);
    trans[1] = -(max_index[1]-cfg.height/2);
    return max_response;
    // pose[2] = 0;
    // std::cout<<"pose: "<<pose.transpose()<<std::endl;
    // return max_response;
}


inline Eigen::ArrayXXcf CorrelationFlow::gaussian_kernel(const Eigen::ArrayXXcf& xf, const Eigen::ArrayXXcf& zf)
{
    unsigned int N = cfg.height * cfg.width;
    auto xx = xf.square().abs().sum()/N; // Parseval's Theorem
    auto zz = zf.square().abs().sum()/N;
    auto zfc = zf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    auto xxzz = (xx+zz-2*xz)/N;
    Eigen::ArrayXXf kernel = (-1/(cfg.sigma*cfg.sigma)*xxzz).exp();
    return FFT(kernel);
}


inline Eigen::ArrayXXcf CorrelationFlow::gaussian_kernel(const Eigen::ArrayXXcf& xf)
{
    unsigned int N = cfg.height * cfg.width;
    auto xx = xf.square().abs().sum()/N; // Parseval's Theorem
    auto zfc = xf.conjugate();
    Eigen::ArrayXXcf xzf = xf * zfc;
    auto xz = IFFT(xzf);
    auto xxzz = (xx+xx-2*xz)/N;
    Eigen::ArrayXXf kernel = (-1/(cfg.sigma*cfg.sigma)*xxzz).exp();
    return FFT(kernel);
}

inline Eigen::ArrayXXf CorrelationFlow::polar(const Eigen::ArrayXXf& array)
{
    cv::Mat polar_img, img=ConvertArrayToMat(array);
    cv::Point2f center((float)img.cols/2, (float)img.rows/2);
    double radius = (double)sqrt((img.rows/2)*(img.rows/2)+(img.cols/2)*(img.cols/2));
    cv::linearPolar(img, polar_img, center, radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
    return ConvertMatToArray(polar_img);
}