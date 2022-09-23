#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

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
