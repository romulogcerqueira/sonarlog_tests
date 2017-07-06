#include <iostream>
#include <deque>
#include <base/samples/Sonar.hpp>
#include "base/test_config.h"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_util/Converter.hpp"
#include "sonar_processing/ScanningHolder.hpp"
#include <opencv2/opencv.hpp>
#include "sonar_processing/ColorGradient.hpp"

using namespace sonar_processing;

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

double calcMSE(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask) {
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
    return (cv::sum(diff)[0] / (cv::countNonZero(mask.total())));
}

double calcPSNR(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& mask) {
    double mse = calcMSE(img1, img2, mask);
    return (10.0 * log10((255 * 255) / mse));
}

int main(int argc, char const *argv[]) {
    // tank images
    cv::Mat cart_real  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_real.yml");
    cv::Mat cart_sim = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_sim.yml");
    cv::Mat cart_mask = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_mask.yml");

    // cart_real.convertTo(cart_real, CV_8U, 255);
    // cart_sim.convertTo(cart_sim, CV_8U, 255);

    // metrics results
    std::cout << "MSE : " << calcMSE(cart_real, cart_sim, cart_mask) << std::endl;
    std::cout << "PSNR: " << calcPSNR(cart_real, cart_sim, cart_mask) << std::endl;

    // display
    cv::Mat out;
    cv::hconcat(cart_real, cart_sim, out);
    cv::imshow("out", out);
    cv::waitKey();
}
