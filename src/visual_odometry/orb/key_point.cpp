#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

void clog_kps(const std::string& prompt, const std::vector<cv::KeyPoint>& kps);

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
    cv::imshow("fast key-points1", img1_show);

    cv::Mat img2_show{};
    cv::drawKeypoints(img2, img2_kps, img2_show, cv::Scalar::all(-1),
         cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("fast key-points2", img2_show);
    clog_kps("fast-kp2", img2_kps);

    // using ORB
    cv::Ptr<cv::ORB> orb = cv::ORB::create(50, 1.2f, 1);
    orb->detect(img1, img1_kps);
    orb->detect(img2, img2_kps);

    cv::drawKeypoints(img1, img1_kps, img1_show);
    cv::drawKeypoints(img2, img2_kps, img2_show);
    cv::imshow("orb-kp1", img1_show);
    cv::imshow("orb-kp2", img2_show);
    clog_kps("orb-kp2", img2_kps);
    cv::waitKey(0);
    cv::imwrite("orb.kp.png", img1_show);
    return 0;
}

void clog_kps(const std::string& prompt, const std::vector<cv::KeyPoint>& kps) {
    std::clog << "================= " << prompt << " =======================\n";
    for (auto& p : kps) {
        std::clog 
            << "angle=" <<  p.angle 
            << " octave=" << p.octave 
            << " class_id=" << p.class_id 
            << " pt=" << p.pt 
            << " response=" << p.response 
            << " size=" << p.size << "\n";
    }
    std::clog << "----------------------------------------------------------\n";
}