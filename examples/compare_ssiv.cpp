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

inline void load_sonar_holder(const base::samples::Sonar &sample, sonar_processing::SonarHolder &sonar_holder)
{
    sonar_holder.Reset(sample.bins,
                       rock_util::Utilities::get_radians(sample.bearings),
                       sample.beam_width.getRad(),
                       sample.bin_count,
                       sample.beam_count);
}

void writeImageToFile(const cv::Mat &frame, const std::string &filename)
{
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    storage << "sonar_image" << frame;
    storage.release();
}

cv::Mat readImageFromFile(const std::string &filename)
{
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    cv::Mat frame;
    storage["sonar_image"] >> frame;
    storage.release();
    return frame;
}

cv::Mat applyColorGradient(const cv::Mat &src, const ColorGradient &colormap)
{
    cv::Mat dst = cv::Mat(src.size(), CV_8UC3);

    float red, green, blue;
    for (size_t i = 0; i < src.rows; i++)
    {
        for (size_t j = 0; j < src.cols; j++)
        {
            // colormap.getColorAtValue((1.0 / 255) * src.at<uchar>(i,j), red, green, blue);
            colormap.getColorAtValue(src.at<float>(i, j), red, green, blue);
            dst.at<cv::Vec3b>(i, j) = cv::Vec3b(blue * 255, green * 255, red * 255);
        }
    }
    return dst;
}

const std::string SAVE_PATH_STRING = std::string("/home/romulo/Desktop/");

int main(int argc, char const *argv[])
{
    ColorGradient colormap;
    colormap.colormapSelector(COLORGRADIENT_HOT);

    // real images
    cv::Mat ssiv_cart_real = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_real.yml");
    cv::Mat ssiv_polar_real = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_real.yml");

    // simulated images
    cv::Mat ssiv_cart_sim = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_sim.yml");
    cv::Mat ssiv_polar_sim = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_sim.yml");

    // mask
    cv::Mat ssiv_cart_mask = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_mask.yml");
    ssiv_cart_mask = 1 - ssiv_cart_mask;

    // apply mask
    ssiv_cart_real += ssiv_cart_mask;
    ssiv_cart_sim += ssiv_cart_mask;

    // output with colormap
    cv::Mat ssiv_cart_real_color = applyColorGradient(ssiv_cart_real, colormap);
    cv::Mat ssiv_polar_real_color = applyColorGradient(ssiv_polar_real, colormap);

    cv::Mat ssiv_cart_sim_color = applyColorGradient(ssiv_cart_sim, colormap);
    cv::Mat ssiv_polar_sim_color = applyColorGradient(ssiv_polar_sim, colormap);

    std::cout << "Size [Cart]: "  << ssiv_cart_sim.rows << "," << ssiv_cart_sim.cols << std::endl;
    std::cout << "Size [Polar]: " << ssiv_polar_sim.rows << "," << ssiv_polar_sim.cols << std::endl;

    // apply mask
    // ssiv_cart_real_color += ssiv_cart_mask;

    cv::imshow("ssiv_cart_real_color", ssiv_cart_real_color);
    cv::imshow("ssiv_polar_real_color", ssiv_polar_real_color);
    cv::imshow("ssiv_cart_sim_color", ssiv_cart_sim_color);
    cv::imshow("ssiv_polar_sim_color", ssiv_polar_sim_color);
    cv::imshow("ssiv_cart_mask", ssiv_cart_mask);

    // new tests
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    cv::imwrite(SAVE_PATH_STRING + "ssiv_cart_real.png", ssiv_cart_real_color, compression_params);
    cv::imwrite(SAVE_PATH_STRING + "ssiv_cart_sim.png", ssiv_cart_sim_color, compression_params);
    cv::imwrite(SAVE_PATH_STRING + "ssiv_polar_real.png", ssiv_polar_real_color, compression_params);
    cv::imwrite(SAVE_PATH_STRING + "ssiv_polar_sim.png", ssiv_polar_sim_color, compression_params);

    cv::waitKey();
    return 0;
}
