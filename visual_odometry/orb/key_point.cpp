#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " img1 img2\n";
        return -1;
    }
    const std::string img1_path = argv[1];
    const std::string img2_path = argv[2];
    cv::Mat img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(img2_path, cv::IMREAD_GRAYSCALE);
    if (!img1.data) {
        std::cerr << "img1 " << img1_path << " load failed\n";
        return -1;
    }
    if (!img2.data) {
        std::cerr << "img2 " << img2_path << " load failed\n";
        return -1;
    }
    const int FAST_THRESHOLD = 40;
    cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(
        FAST_THRESHOLD, true, cv::FastFeatureDetector::TYPE_9_16);
    std::vector<std::vector<cv::KeyPoint>> all_key_points{};
    fast->detect(std::vector<cv::Mat>{img1, img2}, all_key_points);
}