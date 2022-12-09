#include <cmath>

#include <Eigen/Core>

#include "conf.h"

namespace utils {

Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p);
Eigen::Vector2d camera2pixel(const Eigen::Vector3d& c);

bool is_inside_img(const Eigen::Vector2d& p);
template <T>
double calc_bilinear_interpolated(const cv::Mat& img, const Eigen::Vector2d& p);

// inline impl
inline 
Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p) {
    return Eigen::Vector3d(
        (p(0) - conf::CX) / conf::FX,
        (p(1) - conf::CY) / conf::FY,
        1.
    );
}

inline
Eigen::Vector2d camera2pixel(const Eigen::Vector3d& c) {
    auto z = c(2);
    return Eigen::Vector2d(
        c(0) / z * conf::FX + conf::CX,
        c(1) / z * conf::FY + conf::CY
    );
}

inline
bool is_inside_img(const Eigen::Vector2d& p) {
    constexpr int row_lowerbound = conf::IMG_BORDER;
    constexpr int row_upperbound = conf::HEIGHT - conf::IMG_BORDER;
    constexpr int col_lowerbound = conf::IMG_BORDER;
    constexpr int col_upperbound = conf::WIDTH - conf::IMG_BORDER;
    auto x = p(0), y = p(1);
    return (y >= row_lowerbound) && (y < row_upperbound) 
        && (x >= col_lowerbound) && (x < col_upperbound);
}

template <T> 
inline
double calc_bilinear_interpolated(const cv::Mat& img, const Eigen::Vector2d& p) {
    double x = p(0), y = p(1);
    int base_x = static_cast<int>(std::floor(x));
    int base_y = static_cast<int>(std::floor(y));
    double px = x - base_x, py = y - base_y;
    return (1. - px) * (1. - py) * img.at<T>(base_y, base_x)
        + px * (1. - py) * img.at<T>(base_y, base_x + 1) 
        + (1. - px) * py * img.at<T>(base_y + 1, base_x)
        + px * py * img.at<T>(base_y + 1, base_x + 1);
}

} // end of namesapce utils