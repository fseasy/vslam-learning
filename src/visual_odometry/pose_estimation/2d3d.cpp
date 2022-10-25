// Perspective-n-Points
#include <opencv2/opencv.hpp>
#include <src/visual_odometry/orb/orb.h>

#include "pose.h"

std::vector<cv::Point3f> load_depth_and_make_3d_points(const std::string& depth_fpath,
    const std::vector<cv::Point2f>& points2d);

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "param error.\n"
            << "Usage: " << argv[0] << " img1 img1-depth img2\n";
        return 1; 
    }
    std::vector<std::string> img_paths{argv[1], argv[3]};
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
    
    auto img1_3d_points = load_depth_and_make_3d_points(argv[2], 
        matches, match_points.at(0));
}

std::vector<cv::Point3f> load_depth_and_make_3d_points(const std::string& depth_fpath,
    const std::vector<cv::Point2f>& points2d,
    const cv::Mat& K) {
    auto depth_mat = cv::imread(depth_fpath, cv::IMREAD_UNCHANGED);
    std::vector<cv::Point3f> points3d{};
    points3d.reserve(points2d.size());
    for (auto& p : points2d) {
        auto raw_depth = depth_mat.at<ushort>(p.y, p.x);
        if (raw_depth == 0) {
            // bad
            continue;
        }
        // as slambook
        float actual_depth = raw_depth / 5000.f;
        // 深度，其实是相机空间下的；
        // 所以，x，y也需要转换到相机空间下！
        cv::Point3f camera3d_norm = pixel2camera(p, K);
        cv::Point3f camera3d = camera3d_norm * actual_depth;
        points3d.push_back(std::move(camera3d));
    }
    return points3d;
}