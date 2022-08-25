#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "utils/timer.hpp"

bool load_img_and_detect_compute(const std::string& img_path, 
    cv::Ptr<cv::ORB> orb, 
    cv::Mat& img, 
    std::vector<cv::KeyPoint>& kps,
    cv::Mat& descriptors);

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
    cv::Ptr<cv::DescriptorMatcher> bf_matcher = cv::DescriptorMatcher::create(
        cv::DescriptorMatcher::MatcherType::BRUTEFORCE_HAMMING
    );
    std::vector<cv::DMatch> matches{};

    return 0;
}

bool load_img_and_detect_compute(const std::string& img_path, 
    cv::Ptr<cv::ORB> orb, 
    cv::Mat& img, 
    std::vector<cv::KeyPoint>& kps,
    cv::Mat& descriptors) {
    AutoTimer timer("load-img-detect-compute");
    img = cv::imread(img_path, cv::IMREAD_COLOR);
    timer.duration_ms("imread");
    if (!img.data) {
        std::cerr << "Load img " << img_path << " failed\n";
        return false;
    }
    orb->detect(img, kps);
    timer.duration_ms("detect");
    orb->compute(img, kps, descriptors);
    timer.duration_ms("compute");
    return true;
}

void match_and_draw(const std::vetor<cv::Mat>& descriptors, 
    const std::vector<cv::Mat>& imgs,
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    std::vector<cv::DMatch>& matches) {
    AutoTimer timer("match-and-draw");
    bf_matcher->match(descriptors[0], descriptors[1], matches);
    timer.duration_ms("match");
    cv::Mat draw_img{};
    cv::drawMatches(imgs[0], kps[0], imgs[1], kps[1], matches, draw_img);
    cv::imshow("1-1 match result", draw_img);
}