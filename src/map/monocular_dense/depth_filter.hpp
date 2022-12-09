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
    double calc_ncc(
        const cv::Mat& ref_img,
        const cv::Mat& new_img,
        const Eigen::Vector2d& point_ref,
        const Eigen::Vector2d& point_new
    );

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
    // 设置搜索深度的下限
    pnt_depth_min = std::max(pnt_depth_min, 0.1);
    double pnt_depth_max = pnt_depth_mean + range_half_width;

    Eigen::Vector2d pnt_new_mean = utils::camera2pixel(
        T_new_ref * (camera_ref * pnt_depth_mean));
    Eigen::Vector2d pnt_new_min = utils::camera2pixel(
        T_new_ref * (camera_ref * pnt_depth_min));
    Eigen::Vector2d pnt_new_max = utils::camera2pixel(
        T_new_ref * (cemera_ref * pnt_depth_max));
    
    Eigen::Vector2d epipolar_line = pnt_new_max - pnt_new_min;
    double epipolar_half_len = epipolar_line.norm() * 0.5;
    // 设置搜索半径的上限
    epipolar_half_len = std::min(epipolar_half_len, 100.);
    epipolar_direction = epipolar_line.normalized();
    // 开始沿极线搜索
    double best_ncc = -1.;
    Eigen::Vector2d pnt_new_best_match{};
    // step = \sqrt(2) = 0.7
    for (double l = -epipolar_half_len; l <= epipolar_half_len; l += 0.7) {
        Eigen::Vector2d pnt_new_search = pnt_new_mean + epipolar_direction * l;
        if (!utils::is_inside_img(pnt_new_search)) {
            continue;
        }
        double ncc = calc_ncc(ref_img, new_img, point_ref, pnt_new_search);
    }
}

inline
double NaiveDepthFilter::calc_ncc(
        const cv::Mat& ref_img,
        const cv::Mat& new_img,
        const Eigen::Vector2d& point_ref,
        const Eigen::Vector2d& point_new) {
    // 零均值-归一化互相关 (normalized cross correlation)
    using conf::NCC_WINDOW_SIZE;
    constexpr int NCC_AREA = std::pow(NCC_WINDOW_SIZE * 2 + 1, 2);
    
    auto _for_loop_impl = [&]() -> double {
        
        std::vector<double> ref_values{}; ref_values.reserve(NCC_AREA);
        std::vector<double> new_values{}; new_values.reserve(NCC_AREA);
        for (int dy = - NCC_WINDOW_SIZE; dy <= NCC_WINDOW_SIZE; ++dy) {
            for (int dx = - NCC_WINDOW_SIZE; dx <= NCC_WINDOW_SIZE; ++dx) {
                int ref_x = point_ref(0) + dx;
                int ref_y = point_ref(1) + dy;
                // max value = 255, use it to normalize.
                double ref_value = ref_img.at<uchar>(ref_y, ref_x) / 255.;
                ref_values.push_back(ref_value);
                double new_value = utils::calc_bilinear_interpolated<uchar>(
                    new_img, 
                    point_new + Eigen::Vector2d(dx, dy)) / 255.;
                new_values.push_back(new_value);
            }
        }
        
    };
}

} // end of namespace mdf
