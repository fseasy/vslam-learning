// 这里直接用的 OpenCV 的函数；具体推导和内部实现，可以见 
// https://blog.csdn.net/AAAA202012/article/details/117396962
// 易懂 （推导应该就是 多视图几何 书里的推导）

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <src/visual_odometry/orb/orb.h>

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "param error.\n"
            << "Usage: " << argv[0] << " img1 img2\n";
        return 1; 
    }
    std::vector<std::string> img_paths{argv[1], argv[2]};
    std::vector<cv::Mat> imgs(2);
    std::vector<std::vector<cv::KeyPoint>> kps(2);
    std::vector<cv::Mat> descriptors(2);
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    for (auto i = 0; i < 2; ++i) {
        if (!load_img_and_detect_compute(img_paths.at(i), orb, 
            imgs.at(i), kps.at(i), descriptors.at(i))) {
            return 1;
        }
    }
}