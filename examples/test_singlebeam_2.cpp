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

// // simulated sonar image
// int main(int argc, char const *argv[]) {
//     ColorGradient colormap;
//     colormap.colormapSelector(COLORGRADIENT_HOT);
//
//     const std::string logfiles[] = {
//         DATA_PATH_STRING + "/logs/paper/tank_sim.0.log",
//     };
//
//     uint num_logfiles = sizeof(logfiles) / sizeof(std::string);
//     size_t start_index = (argc == 2) ? atoi(argv[1]) : 0;
//     base::Angle last = base::Angle::fromDeg(0);
//
    // for (size_t i = 0; i < num_logfiles; i++) {
    //     rock_util::LogReader reader(logfiles[i]);
    //     rock_util::LogStream stream = reader.stream("sonar_scanning_front.sonar_samples");
    //     stream.set_current_sample_index(start_index);
    //
    //     ScanningHolder holder(588, 588, base::Angle::Min(), base::Angle::Max());
    //
    //     base::samples::Sonar sample;
    //     while (stream.current_sample_index() < stream.total_samples()) {
    //         stream.next<base::samples::Sonar>(sample);
    //
    //         // update scanning holder
    //         holder.update(sample);
    //
    //         // current sonar image
    //         cv::Mat tank_cart_sim = holder.getCartImage();
    //         cv::Mat tank_cart_mask = holder.getCartMask();
    //
    //         writeImageToFile(tank_cart_mask, "/home/romulo/workspace/sonar_toolkit/data/logs/paper/tank/tank_cart_mask.yml");
    //
    //         cv::Mat tank_cart_sim_color = applyColorGradient(tank_cart_sim, colormap);
    //         cv::imshow("tank_cart_sim_color", tank_cart_sim_color);
    //         cv::imshow("tank_cart_mask", tank_cart_mask);
    //         std::cout << "========== IDX   : " << stream.current_sample_index() << std::endl;
    //         cv::waitKey(5);
    //
    //     }
    // }
    // cv::Mat img1 = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_sim.yml");
    // cv::Mat img2 = readImageFromFile(DATA_PATH_STRING + "/logs/paper/tank/tank_cart_mask.yml");
    // cv::imshow("img1", img1);
    // cv::imshow("img2", img2);
    // cv::waitKey();
// }

// // real sonar image
// int main(int argc, char const *argv[]) {
//     ColorGradient colormap;
//     colormap.colormapSelector(COLORGRADIENT_HOT);
//
//     const std::string logfiles[] = {
//         DATA_PATH_STRING + "/logs/paper/tank_micron.0.log",
//     };
//
//     uint num_logfiles = sizeof(logfiles) / sizeof(std::string);
//     size_t start_index = (argc == 2) ? atoi(argv[1]) : 0;
//
//     for (size_t i = 0; i < num_logfiles; i++) {
//         rock_util::LogReader reader(logfiles[i]);
//         rock_util::LogStream stream = reader.stream("sonar.sonar_beam");
//         stream.set_current_sample_index(start_index);
//
//         ScanningHolder holder(588, 588, base::Angle::Min(), base::Angle::Max());
//
//         base::Angle last = base::Angle::fromDeg(0);
//         base::Angle begin = base::Angle::fromDeg(0);
//
//         base::samples::SonarBeam beam;
//         while (stream.current_sample_index() < stream.total_samples()) {
//             stream.next<base::samples::SonarBeam>(beam);
//             base::samples::Sonar sample(beam);
//
//             if(begin.rad == 0) {
//                 begin = sample.bearings[0];
//             }
//
//             // update scanning holder
//             holder.update(sample);
//
//             // current sonar image
//             cv::Mat tank_cart_real = holder.getCartImage();
//
//             if (stream.current_sample_index() == 160) {
//                 writeImageToFile(tank_cart_real, "/home/romulo/workspace/sonar_toolkit/data/logs/paper/tank/tank_cart_real.yml");
//                 exit(0);
//             }
//
//             cv::Mat tank_cart_real_color = applyColorGradient(tank_cart_real, colormap);
//             cv::imshow("tank_cart_real_color", tank_cart_real_color);
//             std::cout << "========== IDX   : " << stream.current_sample_index() << std::endl;
//             std::cout << "========== DIFF  : " << (sample.bearings[0] - last).rad << std::endl;
//             std::cout << "========== BEGIN : " << begin.getDeg() << std::endl;
//             last = sample.bearings[0];
//             cv::waitKey(5);
//         }
//     }
// }

void scanning_build_full_sample() {

}

// real sonar image
int main(int argc, char const *argv[]) {
    ColorGradient colormap;
    colormap.colormapSelector(COLORGRADIENT_HOT);

    const std::string logfiles[] = {
        DATA_PATH_STRING + "/logs/paper/tank_micron.0.log",
    };

    uint num_logfiles = sizeof(logfiles) / sizeof(std::string);
    size_t start_index = (argc == 2) ? atoi(argv[1]) : 0;

    for (size_t i = 0; i < num_logfiles; i++) {
        rock_util::LogReader reader(logfiles[i]);
        rock_util::LogStream stream = reader.stream("sonar.sonar_beam");
        stream.set_current_sample_index(start_index);

        ScanningHolder holder(588, 588, base::Angle::Min(), base::Angle::Max());

        base::Angle last = base::Angle::fromDeg(0);
        base::Angle begin = base::Angle::fromDeg(0);

        base::samples::SonarBeam beam;

        float stepSum = 0;
        base::samples::Sonar fullSample;
        while (stream.current_sample_index() < stream.total_samples()) {
            stream.next<base::samples::SonarBeam>(beam);
            base::samples::Sonar sample(beam);

            // update scanning holder
            holder.update(sample);

            // current sonar image
            cv::Mat tank_cart_real = holder.getCartImage();

            float step  = fabs((sample.bearings[0] - last).rad);
            if(stream.current_sample_index() > 2) {
                stepSum += step;
            }


            cv::Mat tank_cart_real_color = applyColorGradient(tank_cart_real, colormap);
            cv::imshow("tank_cart_real_color", tank_cart_real_color);
            std::cout << "========== IDX   : " << stream.current_sample_index() << std::endl;
            std::cout << "========== DIFF  : " <<  step << std::endl;
            std::cout << "========== CURRENT BEARING: " << sample.bearings[0].getDeg() << std::endl;
            std::cout << "========== stepSum: " << stepSum << std::endl;

            if (fullSample.bin_count == 0) {
                fullSample.bin_count = sample.bins.size();
            }

            fullSample.pushBeam(sample.bins, sample.bearings[0]);
            last = sample.bearings[0];
            cv::waitKey(5);


            if (stepSum > (2 * M_PI)) {
                std::cout << "bearing: " << sample.bearings[0].getDeg() << std::endl;
                std::cout << "Bye bye..." << std::endl;
                break;
            }
        }

        // sonar_processing::SonarHolder sonar_holder;
        // sonar_holder.Reset(fullSample.bins, rock_util::Utilities::get_radians(fullSample.bearings), fullSample.beam_width.getRad(), fullSample.bin_count, fullSample.beam_count);
        cv::Mat out(cv::Size(fullSample.bin_count, fullSample.beam_count), CV_32FC1, &fullSample.bins[0]);

        cv::imshow("output", out);
        cv::waitKey();

    }
}
