#include <iostream>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <base/samples/Sonar.hpp>
// #include <frame_helper/ColorGradient.h>
#include "base/MathUtil.hpp"
#include "base/test_config.h"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_processing/SonarHolder.hpp"
#include "sonar_processing/ColorGradient.hpp"
#include "sonar_processing/QualityMetrics.hpp"
#include "sonar_util/Converter.hpp"
#include "sonarlog_pipeline/Application.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace sonarlog_pipeline;
using namespace sonar_processing;
using namespace sonar_processing::denoising;

inline void load_sonar_holder(const base::samples::Sonar& sample, sonar_processing::SonarHolder& sonar_holder) {
    sonar_holder.Reset(sample.bins,
        rock_util::Utilities::get_radians(sample.bearings),
        sample.beam_width.getRad(),
        sample.bin_count,
        sample.beam_count);
}

void writeImageToFile(const cv::Mat& frame, const std::string& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    storage << "sonar_image" << frame;
    storage.release();
}

cv::Mat readImageFromFile(const std::string& filename) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    cv::Mat frame;
    storage["sonar_image"] >> frame;
    storage.release();
    return frame;
}

cv::Mat applyColorGradient(const cv::Mat& src, const ColorGradient& colormap) {
    cv::Mat dst = cv::Mat(src.size(), CV_8UC3);

    float red, green, blue;
    for (size_t i = 0; i < src.rows; i++) {
        for (size_t j = 0; j < src.cols; j++) {
            // colormap.getColorAtValue((1.0 / 255) * src.at<uchar>(i,j), red, green, blue);
            colormap.getColorAtValue(src.at<float>(i,j), red, green, blue);
            dst.at<cv::Vec3b>(i,j) = cv::Vec3b(blue * 255, green * 255, red * 255);
        }
    }
    return dst;
}

double calcMSE(const cv::Mat& img1, const cv::Mat& img2) {
    // CV_Assert(img1.type() == CV_32FC1 && img2.type() == CV_32FC1 && mask.type() == CV_8U);
    // double sum = 0;
    // for (size_t i = 0; i < img1.cols; i++) {
    //     for (size_t j = 0; j < img1.rows; j++) {
    //         if (mask.at<uchar>(j,i)) {
    //             float diff = img1.at<float>(j,i) - img2.at<float>(j,i);
    //             sum += (diff * diff);
    //         }
    //     }
    // }
    cv::Mat diff;
    cv::absdiff(img1, img2, diff);
    diff = diff.mul(diff);
    return (cv::sum(diff)[0] / (img1.total()));
}

double calcPSNR(const cv::Mat& img1, const cv::Mat& img2) {
    double mse = calcMSE(img1, img2);
    return (10.0 * log10((255 * 255) / mse));
}

int main(int argc, char const *argv[]) {
    ColorGradient colormap;
    colormap.colormapSelector(COLORGRADIENT_HOT);

    // real images
    cv::Mat ssiv_cart_real  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_real.yml");
    cv::Mat ssiv_polar_real = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_real.yml");

    // simulated images
    cv::Mat ssiv_cart_sim   = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_sim.yml");
    cv::Mat ssiv_polar_sim  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_sim.yml");

    // output with colormap
    cv::Mat ssiv_cart_real_color  = applyColorGradient(ssiv_cart_real, colormap);
    cv::Mat ssiv_polar_real_color = applyColorGradient(ssiv_polar_real, colormap);

    cv::Mat ssiv_cart_sim_color  = applyColorGradient(ssiv_cart_sim, colormap);
    cv::Mat ssiv_polar_sim_color = applyColorGradient(ssiv_polar_sim, colormap);

    cv::imshow("ssiv_cart_real_color", ssiv_cart_real_color);
    cv::imshow("ssiv_polar_real_color", ssiv_polar_real_color);
    cv::imshow("ssiv_cart_sim_color", ssiv_cart_sim_color);
    cv::imshow("ssiv_polar_sim_color", ssiv_polar_sim_color);

    // ssiv_polar_real.convertTo(ssiv_polar_real, CV_8U, 255);
    // ssiv_polar_sim.convertTo(ssiv_polar_sim, CV_8U, 255);

    // quality evaluation results
    std::cout << "MSE   : " << calcMSE(ssiv_polar_real, ssiv_polar_sim) << std::endl;
    // std::cout << "RMSE  : " << qs::RMSE(ssiv_polar_real, ssiv_polar_sim) << std::endl;
    // std::cout << "PSNR  : " << qs::PSNR(ssiv_polar_sim, ssiv_polar_real) << std::endl;
    // std::cout << "SSIM  : " << qs::SSIM(ssiv_polar_real, ssiv_polar_sim) << std::endl;
    // std::cout << "MSSIM : " << qs::MSSIM(ssiv_polar_real, ssiv_polar_sim) << std::endl;
    // std::cout << "UQI   : " << qs::UQI(ssiv_polar_real, ssiv_polar_sim) << std::endl;

    cv::waitKey();
    return 0;
}
