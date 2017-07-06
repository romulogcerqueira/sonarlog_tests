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

    const std::string logfiles[] = {
        // "/home/romulo/workspace/sonar_toolkit/data/logs/paper/ssiv_sim.1.log",
        DATA_PATH_STRING + "/logs/paper/ssiv_gemini.1.log"
    };

    uint num_logfiles = sizeof(logfiles) / sizeof(std::string);
    sonar_processing::SonarHolder sonar_holder;
    base::samples::Sonar sample;
    size_t start_index = (argc == 2) ? atoi(argv[1]) : 0;
    double scale_factor = 0.4;

    for (size_t i = 0; i < num_logfiles; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("gemini.sonar_samples");
        // rock_util::LogStream stream = reader.stream("sonar_multibeam_imager.sonar_samples");
        stream.set_current_sample_index(start_index);

        // while (stream.current_sample_index() < stream.total_samples()) {
            stream.next<base::samples::Sonar>(sample);
            load_sonar_holder(sample, sonar_holder);

            /* polar image */
            cv::Mat polar(sample.beam_count, sample.bin_count, CV_32FC1, (float*) sample.bins.data());

            /* cartesian image */
            cv::Mat cart = sonar_holder.cart_image();
            cv::resize(cart, cart, cv::Size(862, 500));

            /* file operations */
            // writeImageToFile(polar, "/home/romulo/workspace/sonar_toolkit/data/logs/paper/storage/ssiv_sim_polar.yml");
            // writeImageToFile(cart, "/home/romulo/workspace/sonar_toolkit/data/logs/paper/storage/ssiv_sim_cart.yml");
            // writeImageToFile(polar, "/home/romulo/workspace/sonar_toolkit/data/logs/paper/storage/ssiv_real_polar.yml");
            // writeImageToFile(cart, "/home/romulo/workspace/sonar_toolkit/data/logs/paper/storage/ssiv_real_cart.yml");
            // cv::Mat out1 = readImageFromFile("real.yml");
            // cv::Mat out2 = readImageFromFile("real.yml");

            /* output */
            cv::imshow("polar", polar);
            cv::imshow("cart", cart);
            cv::waitKey();

            // std::cout << "Overlap: " << evaluateOverlap(out1, out2) << std::endl;

            // std::cout << " === IDX === " << stream.current_sample_index() << std::endl;
            // std::cout << sample.bin_count << std::endl;
        // }
    }

return 0;
}

// int main(int argc, char const *argv[]) {
//     ColorGradient colormap;
//     colormap.colormapSelector(COLORGRADIENT_HOT);
//
//     cv::Mat ssiv_real  = readImageFromFile("ssiv_real.yml");
//     cv::Mat ssiv_sim   = readImageFromFile("ssiv_sim.yml");
//     cv::Mat sonar_mask = readImageFromFile("sonar_mask.yml");
//
//     // convert to 8-bit
//     // ssiv_real.convertTo(ssiv_real, CV_8U, 255);
//     // ssiv_sim.convertTo(ssiv_sim, CV_8U, 255);
//
//     // output with colormap
//     cv::Mat ssiv_real_color = applyColorGradient(ssiv_real, colormap);
//     cv::Mat ssiv_sim_color  = applyColorGradient(ssiv_sim, colormap);
//
//     cv::imshow("ssiv_real_color", ssiv_real_color);
//     cv::imshow("ssiv_sim_color", ssiv_sim_color);
//
//     // evaluation results
//     // std::cout << "Overlap Real/Sim: " << (evaluateOverlap(ssiv_real, ssiv_sim, sonar_mask) * 100) << "% " << std::endl;
//
//     cv::waitKey();
//
//     return 0;
// }
