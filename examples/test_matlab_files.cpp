#include <iostream>
#include <cmath>
#include <algorithm>
#include <time.h>
#include <fstream>
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

void writeCSV(std::string filename, cv::Mat m) {
   cv::Formatter const * c_formatter(cv::Formatter::get("CSV"));
   std::ofstream myfile;
   myfile.open(filename.c_str());
   c_formatter->write(myfile, m);
   myfile.close();
}

int main(int argc, char const *argv[]) {
    // ColorGradient colormap;
    // colormap.colormapSelector(COLORGRADIENT_HOT);

    // // real images (ssiv)
    // cv::Mat ssiv_cart_real  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_real.yml");
    // cv::Mat ssiv_polar_real = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_real.yml");
    //
    // // simulated images (ssiv)
    // cv::Mat ssiv_cart_sim   = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_sim.yml");
    // cv::Mat ssiv_polar_sim  = readImageFromFile(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_sim.yml");

    // real images (tank)
    // cv::Mat tank_cart_real = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_real.yml");
    cv::Mat tank_polar_real = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_polar_real.yml");

    // simulated images (tank)
    // cv::Mat tank_cart_sim = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_sim.yml");
    cv::Mat tank_polar_sim = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_polar_sim.yml");
    // cv::Mat tank_cart_mask = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_mask.yml");

    // pass to Matlab (ssiv)
    // writeCSV(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_real.csv",  ssiv_cart_real);
    // writeCSV(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_real.csv", ssiv_polar_real);
    // writeCSV(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_cart_sim.csv",   ssiv_cart_sim);
    // writeCSV(DATA_PATH_STRING + "/logs/paper/ssiv/ssiv_polar_sim.csv",  ssiv_polar_sim);
    // writeCSV(DATA_PATH_STRING + "/logs/paper/ssiv/tank_cart_real.csv",  tank_cart_real);

    // pass to Matlab (tank)
    // writeCSV(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_real.csv",  tank_cart_real);
    // writeCSV(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_sim.csv",  tank_cart_sim);
    // writeCSV(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_mask.csv",  tank_cart_mask);
    writeCSV(DATA_PATH_STRING + "/logs/paper/tank/tank_polar_real.csv",  tank_polar_real);
    writeCSV(DATA_PATH_STRING + "/logs/paper/tank/tank_polar_sim.csv",  tank_polar_sim);

    return 0;
}
