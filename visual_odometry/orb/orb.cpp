#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

void load_img_and_detect(const std::string& img_path, 
    cv::Ptr<cv::ORB> orb, 
    cv::Mat& img, 
    std::vector<cv::KeyPoint>& kps);

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "usage: " << argv[0] << " img1 img2\n";
        return -1;
    }
    const std::string img1_path = argv[1];
    const std::string img2_path = argv[2];
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8);
    std::pair<cv::Mat, cv::Mat> img_pair{};
 
    return 0;
}

bool load_img_and_detect(const std::string& img_path, 
    cv::Ptr<cv::ORB> orb, 
    cv::Mat& img, 
    std::vector<cv::KeyPoint>& kps) {
    img = cv::imread(img_path, cv::IMREAD_COLOR);
    if (!img.data) {
        std::cerr << "Load img " << img_path << " failed\n";
        return false;
    }
    orb->detect(img, kps);
    return true;
}