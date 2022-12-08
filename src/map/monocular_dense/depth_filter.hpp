#include <cmath>

#include <opencv2/opencv.hpp>
#include <sophus/se2.hpp>

#include "utils.hpp"

namespace mdf {

class NaiveDepthFilter {
public:
    NaiveDepthFilter(int height, int width, double depth, double cov);

    void update(const cv::Mat& ref_img, const cv::Mat& new_img, 
        const Sophus::SE3d& T_new_ref);

private:
    void epipolar_search(
        const cv::Mat& ref_img, 
        const cv::Mat& new_img,
        const Eigen::Vector2d& point_ref, 
        const Sophus::SE3d& T_new_ref,
        Eigen::Vector2d& point_new, Eigen::Vector2d& epipolar_direction);

private:
    cv::Mat depth_;
    cv::Mat cov_;
};

// inline impl

inline
NaiveDepthFilter::NaiveDepthFilter(
    int height, int width, double depth, double cov)
    : depth_(height, width, CV_64F, depth),
    cov_(height, width, CV_64F, cov) {}

inline 
void NaiveDepthFilter::update(const cv::Mat& ref_img, const cv::Mat& new_img, 
        const Sophus::SE3d& T_new_ref) {
    auto _B = conf::IMG_BORDER;
    for (int y = _B; y < ref_img.rows - _B; ++y) {
        for (int x = _B; x < ref_img.cols - _B; ++x ) {
            auto pnt_cov = cov_.at<double>(y, x);
            // 收敛/发散
            if (pnt_cov < conf::MIN_COV || pnt_cov > conf::MAX_COV) {
                continue;
            }
            // 极线搜索
            Eigen::Vector2d pnt_new{};
            Eigen::Vector2d epipolar_direction{};
        }

    }
}

void NaiveDepthFilter::epipolar_search(
    const cv::Mat& ref_img, 
    const cv::Mat& new_img,
    const Eigen::Vector2d& point_ref, 
    const Sophus::SE3d& T_new_ref,
    Eigen::Vector2d& point_new, Eigen::Vector2d& epipolar_direction) {
    Eigen::Vector3d camera_ref = utils::pixel2camera(point_ref);
    camera_ref.normalize();
    // set search range
    int y = static_cast<int>(point_ref(1));
    int x = static_cast<int>(point_ref(0));
    double pnt_depth_mean = depth_.at<double>(y, x);
    double pnt_cov = cov_.at<double>(y, x);
    double pnt_std_dev = std::sqrt(pnt_cov);
    double range_half_width = 3 * pnt_std_dev;
    double pnt_depth_min = pnt_depth_mean - range_half_width;
    pnt_depth_mean = std::max(pnt_depth_mean, 0.1);
    double pnt_depth_max = pnt_depth_mean + range_half_width;

    Eigen::Vector2d pnt_new_mean = utils::camera2pixcel(
        T_new_ref * (camera_ref * pnt_depth_mean));
    

}

} // end of namespace mdf
