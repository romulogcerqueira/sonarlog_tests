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

double evaluateOverlap(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask) {
    cv::Mat overlap = cv::Mat::zeros(img1.size(), CV_8U);
    for (size_t i = 0; i < img1.rows; i++) {
        for (size_t j = 0; j < img1.cols; j++) {
            if (mask.at<uchar>(i,j) && (img1.at<uchar>(i,j) == img2.at<uchar>(i,j)))
                overlap.at<uchar>(i,j) = 255;
        }
    }
    cv::imshow("overlap", overlap);
    int matches = cv::countNonZero(overlap);
    return ((double) matches / overlap.size().area());
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

int main(int argc, char const *argv[]) {
    ColorGradient colormap;
    colormap.colormapSelector(COLORGRADIENT_HOT);

    // real images
    cv::Mat ssiv_real_cart  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/storage/ssiv_real_cart.yml");
    cv::Mat ssiv_real_polar = readImageFromFile(DATA_PATH_STRING + "/logs/paper/storage/ssiv_real_polar.yml");

    // simulated images
    cv::Mat ssiv_sim_cart   = readImageFromFile(DATA_PATH_STRING + "/logs/paper/storage/ssiv_sim_cart.yml");
    cv::Mat ssiv_sim_polar  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/storage/ssiv_sim_polar.yml");

    // output with colormap
    cv::Mat ssiv_real_color_cart  = applyColorGradient(ssiv_real_cart, colormap);
    cv::Mat ssiv_real_color_polar = applyColorGradient(ssiv_real_polar, colormap);

    cv::Mat ssiv_sim_color_cart  = applyColorGradient(ssiv_sim_cart, colormap);
    cv::Mat ssiv_sim_color_polar = applyColorGradient(ssiv_sim_polar, colormap);

    cv::imshow("ssiv_real_color_cart", ssiv_real_color_cart);
    cv::imshow("ssiv_real_color_polar", ssiv_real_color_polar);
    cv::imshow("ssiv_sim_color_cart", ssiv_sim_color_cart);
    cv::imshow("ssiv_sim_color_polar", ssiv_sim_color_polar);

    // quality evaluation results
    // std::cout << "MSE  :" << qs::MSE(ssiv_real, ssiv_sim) << std::endl;
    // std::cout << "RMSE :" << qs::RMSE(ssiv_real, ssiv_sim) << std::endl;
    // std::cout << "PSNR :" << qs::PSNR(ssiv_real, ssiv_sim) << std::endl;
    // std::cout << "MSSIM:" << qs::MSSIM(ssiv_real, ssiv_sim) << std::endl;
    // std::cout << "UQI  :" << qs::UQI(ssiv_real, ssiv_sim) << std::endl;

    cv::waitKey();
    return 0;
}
