// 这里直接用的 OpenCV 的函数；具体推导和内部实现，可以见 
// https://blog.csdn.net/AAAA202012/article/details/117396962
// 易懂 （推导应该就是 多视图几何 书里的推导）

#include <iostream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <src/visual_odometry/orb/orb.h>

#include "pose.h"

void epipolar_pose_estimate(
    const std::vector<std::vector<cv::Point2f>>& match_points,
    const cv::Mat& K,
    cv::Mat& R,
    cv::Mat& t,
    std::vector<std::vector<cv::Point2f>>& inlier_match_points);

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
    std::vector<cv::DMatch> matches{};
    if (match_and_draw(descriptors, imgs, kps, matches)) {
        cv::waitKey(1);
    }
    std::vector<std::vector<cv::Point2f>> match_points = get_match_points(
        kps, matches);
    
    const cv::Mat K = (cv::Mat_<double>(3, 3) << 
        520.9, 0, 325.1, 
        0, 521.0, 249.7, 
        0, 0, 1
    );
    cv::Mat R;
    cv::Mat t;
    std::vector<std::vector<cv::Point2f>> inlier_match_points;
    epipolar_pose_estimate(match_points, K, R, t, inlier_match_points);

}

void epipolar_pose_estimate(
    const std::vector<std::vector<cv::Point2f>>& match_points,
    const cv::Mat& K,
    cv::Mat& R,
    cv::Mat& t,
    std::vector<std::vector<cv::Point2f>>& inlier_match_points) {

    cv::Mat inlier_indicator{};
    cv::Mat E = cv::findEssentialMat(match_points.at(0), match_points.at(1),
        K, cv::RANSAC, 0.999, 1.0, inlier_indicator);
    int cheirality_check_pnt_num = cv::recoverPose(E, 
        match_points.at(0), match_points.at(1), K, 
        R, t, inlier_indicator);
    std::size_t inlier_sz = cv::sum(inlier_indicator)(0);
    std::clog << "Total match point pairs = " << match_points.size()
        << " inlier point pairs = " << inlier_sz
        << " cheilarity check point num = " << cheirality_check_pnt_num 
        << "\n";
    inlier_match_points.resize(2);
    inlier_match_points.at(0).reserve(inlier_sz);
    inlier_match_points.at(1).reserve(inlier_sz);
    for (std::size_t i = 0U; i < match_points.size(); ++i) {
        int is_inlier = inlier_indicator.at<int>(i);
        if (!is_inlier) {
            continue;
        }
        inlier_match_points.at(0).push_back(match_points.at(0).at(i));
        inlier_match_points.at(1).push_back(match_points.at(1).at(i));
    }
}
