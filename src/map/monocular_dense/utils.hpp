#include <Eigen/Core>

#include "conf.h"

namespace utils {

Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p);
Eigen::Vector2d camera2pixel(const Eigen::Vector3d& c);

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


} // end of namesapce utils