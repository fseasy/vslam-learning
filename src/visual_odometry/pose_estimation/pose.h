#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

inline
std::vector<std::vector<cv::Point2f>> get_match_points(
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    const std::vector<cv::DMatch>& matches) {
    std::vector<std::vector<cv::Point2f>> match_points(2);
    auto sz = matches.size();
    auto& first_pnts = match_points.at(0);
    auto& second_pnts = match_points.at(1);
    first_pnts.reserve(sz);
    second_pnts.reserve(sz);
    for (auto& match : matches) {
        first_pnts.push_back(kps.at(0).at(match.queryIdx).pt);
        second_pnts.push_back(kps.at(1).at(match.trainIdx).pt);
    }
    return match_points;
}

inline
cv::Point3f pixel2camera(const cv::Point2f& p, const cv::Mat& K) {
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    return cv::Point3f(
        (p.x - cx) / fx,
        (p.y - cy) / fy,
        1.f
    );
};