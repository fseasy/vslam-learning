#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <src/utils/timer.hpp>
#include "orb.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " img1 img2\n";
        return -1;
    }
    std::vector<std::string> img_paths{argv[1], argv[2]};
    cv::Ptr<cv::ORB> orb = cv::ORB::create(200, 1.2f, 8);
    std::vector<cv::Mat> imgs(2);
    std::vector<std::vector<cv::KeyPoint>> kps(2);
    std::vector<cv::Mat> descriptors(2);
    for (std::size_t i = 0U; i < 2U; ++i) {
        bool ok = load_img_and_detect_compute(img_paths[i], 
                orb, imgs[i], kps[i], descriptors[i]); 
        auto& d = descriptors[i];
        std::clog << "descriptor size = (row:" << d.rows << ", cols:"
            << d.cols << ", channel:" << d.channels() 
            << ") type = " << d.type()
            << "\n";
        if (!ok) { return -1; }
    }

    std::vector<cv::DMatch> matches{};
    match_and_draw(descriptors, imgs, kps, matches);
    cv::waitKey(0);
    return 0;
}
