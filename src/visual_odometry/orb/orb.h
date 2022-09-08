#pragma once 

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <src/utils/timer.hpp>

bool load_img_and_detect_compute(const std::string& img_path, 
    cv::Ptr<cv::ORB> orb, 
    cv::Mat& img, 
    std::vector<cv::KeyPoint>& kps,
    cv::Mat& descriptors);

bool match_and_draw(const std::vector<cv::Mat>& descriptors, 
    const std::vector<cv::Mat>& imgs,
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    std::vector<cv::DMatch>& matches);


/*******  impl  ************/
inline
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

inline
bool match_and_draw(const std::vector<cv::Mat>& descriptors, 
    const std::vector<cv::Mat>& imgs,
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    std::vector<cv::DMatch>& matches) {
    cv::Ptr<cv::DescriptorMatcher> bf_matcher = cv::DescriptorMatcher::create(
        cv::DescriptorMatcher::MatcherType::BRUTEFORCE_HAMMING);
    AutoTimer timer("match-and-draw");
    bf_matcher->match(descriptors[0], descriptors[1], matches);
    timer.duration_ms("match");
    std::cerr << "Matches size = " << matches.size() << "\n";
    if (matches.empty()) { return false; }
    cv::Mat draw_img{};
    cv::drawMatches(imgs[0], kps[0], imgs[1], kps[1], matches, draw_img);
    cv::imshow("1-1 match result", draw_img);
    cv::imwrite("all_match.png", draw_img);

    auto [min_match, max_match] = std::minmax_element(matches.begin(), matches.end());
    std::cerr << "Matches min distance = " << min_match->distance
        << ", max distance = " << max_match->distance << "\n"; 
    float threshold = std::max<float>(min_match->distance * 2, 30);
    matches.erase(
        std::remove_if(matches.begin(), matches.end(), 
            // > or >=, will influence the final match points, and then the E, R, t
            [threshold](const cv::DMatch& m){ return m.distance > threshold; }
        ),
        matches.end()
    );
    std::cerr << "Good matches size = " << matches.size() << "\n";
    cv::drawMatches(imgs[0], kps[0], imgs[1], kps[1], matches, draw_img);
    cv::imshow("good match result", draw_img);
    cv::imwrite("good_match.png", draw_img);
    return true;
}