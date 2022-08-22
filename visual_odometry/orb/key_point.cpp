#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "cv_helper.hpp"

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
    std::vector<cv::KeyPoint>& img1_kps = all_key_points[0];
    std::vector<cv::KeyPoint>& img2_kps = all_key_points[1];
    cv::Mat img1_show{};
    cv::drawKeypoints(img1, img1_kps, img1_show, cv::Scalar::all(-1),  
        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("key-points1", img1_show);
    for (auto& p : img1_kps) {
        std::clog 
            << p.angle << " " << p.octave << " " 
            << p.class_id << " " << p.pt << " " << p.response << " "
            << p.size << "\n";
    }
    cv::Mat img2_show{};
    cv::drawKeypoints(img2, img2_kps, img2_show, cv::Scalar::all(-1),
         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("key-points2", img2_show);

    // using ORB
    cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 1);
    orb->detect(img1, img1_kps);
    orb->detect(img2, img2_kps);

    cv::drawKeypoints(img1, img1_kps, img1_show);
    cv::drawKeypoints(img2, img2_kps, img2_show);
    cv::imshow("orb-kp1", img1_show);
    cv::imshow("orb-kp2", img2_show);
    cv::waitKey(0);
    return 0;
}