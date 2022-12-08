#include <Eigen/Core>

#include "conf.h"

namespace utils {

Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p);
Eigen::Vector2d camera2pixel(const Eigen::Vector3d& c);

bool is_inside_img(const Eigen::Vector2d& p);

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


} // end of namesapce utils