#include <opencv/opencv.hpp>

void CalcIntensityCentroid(const cv::Mat& img， 
    const cv::KeyPoint& patch_center, 
    std::size_t patch_size) {
    std::size_t m00 = 0U;
    std::size_t m01 = 0U;
    std::size_t m10 = 0U;
    int patch_half_size = static_cast<int>(patch_size / 2);
    
}