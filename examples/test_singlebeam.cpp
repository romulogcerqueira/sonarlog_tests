#include <iostream>
#include <deque>
#include <base/samples/Sonar.hpp>
#include "base/test_config.h"
#include "rock_util/LogReader.hpp"
#include "rock_util/SonarSampleConverter.hpp"
#include "rock_util/Utilities.hpp"
#include "sonar_util/Converter.hpp"
#include "sonar_processing/ScanningHolder.hpp"
#include "sonar_processing/SonarHolder.hpp"
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

void scanning_build_full_sample(rock_util::LogStream& stream, base::samples::Sonar& result_sample, bool convert = true) {
    base::Angle last = base::Angle::fromDeg(0);
    base::samples::SonarBeam beam;
    base::samples::Sonar sample;

    float stepSum = 0;
    while (stream.current_sample_index() < stream.total_samples()) {
        if(convert) {
            stream.next<base::samples::SonarBeam>(beam);
            sample = base::samples::Sonar(beam);
        } else {
            stream.next<base::samples::Sonar>(sample);
        }

        float step  = fabs((sample.bearings[0] - last).rad);

        if(stream.current_sample_index() > 2) {
            stepSum += step;
        }

        if (result_sample.bin_count == 0) {
            result_sample.bin_count = sample.bins.size();
        }

        result_sample.pushBeam(sample.bins, sample.bearings[0]);
        last = sample.bearings[0];
        if (stepSum > (2 * M_PI)) {
            break;
        }
    }
}

void resulting_sonar_sim(const base::samples::Sonar& real, const base::samples::Sonar &sim, base::samples::Sonar& result) {
    result.bin_count = sim.bin_count;
    for (size_t i = 0; i < real.beam_count; i++) {
        std::vector<float> diffs(sim.beam_count, 0);
        base::Angle real_beam = real.bearings[i];

        for (size_t j = 0; j < sim.beam_count; j++) diffs[j] = fabs((real_beam - sim.bearings[j]).rad);

        std::vector<float>::iterator it = std::min_element(diffs.begin(), diffs.end());
        int beam_idx = std::distance(diffs.begin(), it);
        int index = beam_idx * sim.bin_count;

        std::vector<float> bins;
        std::copy(sim.bins.begin()+index, sim.bins.begin()+index+sim.bin_count, std::back_inserter(bins));
        result.pushBeam(bins, real.bearings[i]);
    }
}

// real sonar image
int main(int argc, char const *argv[]) {

    // real data
    rock_util::LogReader reader_real(DATA_PATH_STRING + "/logs/paper/tank_micron.0.log");
    rock_util::LogStream stream_real = reader_real.stream("sonar.sonar_beam");
    base::samples::Sonar sample_real;
    scanning_build_full_sample(stream_real, sample_real, true);
    cv::Mat out_real(cv::Size(sample_real.bin_count, sample_real.beam_count), CV_32FC1, &sample_real.bins[0]);

    // sim data
    rock_util::LogReader reader_sim(DATA_PATH_STRING + "/logs/paper/tank_sim.0.log");
    rock_util::LogStream stream_sim = reader_sim.stream("sonar_scanning_front.sonar_samples");
    base::samples::Sonar sample_sim;
    scanning_build_full_sample(stream_sim, sample_sim, false);
    cv::Mat out_sim(cv::Size(sample_sim.bin_count, sample_sim.beam_count), CV_32FC1, &sample_sim.bins[0]);

    // resulting
    base::samples::Sonar sample_result;
    resulting_sonar_sim(sample_real, sample_sim, sample_result);
    cv::Mat out_result(cv::Size(sample_result.bin_count, sample_result.beam_count), CV_32FC1, &sample_result.bins[0]);

    out_real = out_real(cv::Rect(1, 0, out_real.cols-2, out_real.rows));

    // file operations
    writeImageToFile(out_real,   DATA_PATH_STRING + "/logs/paper/tank/tank_polar_real.yml");
    writeImageToFile(out_result, DATA_PATH_STRING + "/logs/paper/tank/tank_polar_sim.yml");

    cv::imshow("out_real", out_real);
    cv::imshow("out_sim", out_sim);
    cv::imshow("out_result", out_result);
    cv::waitKey();

    exit(0);
}
