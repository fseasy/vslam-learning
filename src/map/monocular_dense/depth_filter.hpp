#include <cmath>
#include <numeric>

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
        Eigen::Vector2d& point_new, 
        Eigen::Vector2d& epipolar_direction,
        double& match_ncc);
    double calc_ncc(
        const cv::Mat& ref_img,
        const cv::Mat& new_img,
        const Eigen::Vector2d& point_ref,
        const Eigen::Vector2d& point_new
    );
    void update_depth(
        const Eigen::Vector2d& point_ref,
        const Eigen::Vector2d& point_new,
        const Sophus::SE3d& T_new_ref,
        const Eigen::Vector2d& epipolar_direction
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
            double ncc = 0.;
            epipolar_search(ref_img, new_img, Eigen::Vector2d(x, y), T_new_ref,
                pnt_new, epipolar_direction, ncc);
            if (ncc < 0.85) {
                std::clog << "ncc is small, not valid match\n";
                continue;
            }
            
        }

    }
}

void NaiveDepthFilter::epipolar_search(
    const cv::Mat& ref_img, 
    const cv::Mat& new_img,
    const Eigen::Vector2d& point_ref, 
    const Sophus::SE3d& T_new_ref,
    Eigen::Vector2d& point_new, 
    Eigen::Vector2d& epipolar_direction,
    double& match_ncc) {
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
        if (ncc > best_ncc) {
            best_ncc = ncc;
            pnt_new_best_match = pnt_new_search;
        }
    }
    point_new = pnt_new_best_match;
    match_ncc = best_ncc;
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
        double value_size = static_cast<double>(ref_values.size());
        double ref_mean = std::reduce(std::execution::par, 
            ref_values.begin(), ref_values.end()) / value_size;
        double new_mean = std::reduce(std::execution::par,
            new_values.begin(), new_values.end()) / value_size;
        double numerator = 0.;
        double denominator1 = 0., demoninator2 = 0.;
        for (std::size_t i = 0U; i < ref_values.size(); ++i) {
            auto ref1 = ref_values.at(i) - ref_mean;
            auto new1 = new_values.at(i) - new_mean;
            numerator += ref1 * new1;
            denominator1 += ref1 * ref1;
            denominator2 += new1 * new1;
        }
        auto ncc_val = numerator 
            / (std::sqrt(denominator1 * denominator2) + 1e-10);
        return ncc_val;
    };

    return _for_loop_impl();
}

void NaiveDepthFilter::update_depth(
    const Eigen::Vector2d& point_ref,
    const Eigen::Vector2d& point_new,
    const Sophus::SE3d& T_new_ref,
    const Eigen::Vector2d& epipolar_direction
) {
    Eigen::Vector3d f_ref = utils::pixel2camera(point_ref);
    f_ref.normalize();
    Eigen::Vector3d f_new = utils::pixel2camera(point_new);
    f_new.normalize();

    /**
     * formular:
     *  d_ref * f_ref =  R_ref_new * (d_new * f_new) + t_ref_new
     *  d_ref, f_ref 都是常量，乘法中可以随意移动位置； 移项到左边 
     *  =>
     *  f_ref * d_ref - (R_ref_new * f_new) * d_new = t_ref_new
     *  令 f_newT = R_ref_new * f_new， 可写成矩阵形式的乘法
     * =>
     * [ f_ref; f_newT ] * [ d_ref]  = [t_ref_new]
     *                     [- d_new] 
     *  == 3x2           * 2x1        = 3x1
     *  两边左乘一个 [f_ref; f_newT]^T , 得到的结果，其实就是高博的那个公式了。
     *  然后再求逆，乘上b.
     *  也就是： Ax = b, A 不是方阵，于是 1. A^TAx = A^Tb 2. x = (A^TA)^{-1}A^Tb
     *      而 (A^TA)^{-1}A 其实就是 左广义逆矩阵 的一种结果，此解法算是广义逆的应用：
     *      对 Ax = b, 其所有解为 x = A^{g}b - [I - A^{g}A]w
     *      其中 A^{g} 就是A的任一的广义逆矩阵。
     *      参见： https://zh.wikipedia.org/zh-cn/%E5%B9%BF%E4%B9%89%E9%80%86%E9%98%B5
     */

    Sophus::SE3d T_ref_new = T_new_ref.inverse();
    Eigen::Vector3d f_newT = T_ref_new.so3() * f_newT;
    Eigen::Matrixd<3, 2> A{};
    A.col(0) << f_ref;
    A.col(1) << f_newT;
    Eigen::Vector3d b = T_ref_new.translation();
    Eigen::Vector2d x = (A.tanspose() * A).inverse() * A.transpose() * b;
    double d_ref = x(0);
    double d_new = - x(1);
    Eigen::Vector3d P_ref = f_ref * d_ref;
    Eigen::Vector3d P_new_in_ref_coor = f_newT * d_new + b;
    
}


} // end of namespace mdf
