// Perspective-n-Points
/*
---> 对比 PnP 和 epipolar 的结果：
============ cv::SolvePnP result ==============
rvec = [-0.02712017276505522, 0.04060407996706972, 0.05041040454892387]; 

R = [0.9979059096334197, -0.05091940168154324, 0.0398874673436509;
 0.04981866392733154, 0.998362316024705, 0.02812092932821706;
 -0.04125404517368886, -0.02607490123050583, 0.9988083916761146]

tvec = [-0.1267821337944245, -0.008439477762411572, 0.06034934504116969]
============= end cv::SolvePnP ================

=========== 2d2d result =============
R = [0.9979655126814051, -0.0525460114272234, 0.03610750865998041;
 0.05130248546022501, 0.9980855456896863, 0.03454415248525401;
 -0.03785353991562529, -0.03262146790705769, 0.9987506942913453]
 
t = [-0.9295837998982542, -0.1479623532712804, 0.3376108721311374]
=========== end 2d2d result ==========

对比： R 的确差不多，但在小数点第二位就有一定的差别了。
t肯定不同，但各个值并不是成倍数的（比值为 [7.332135625714912, 17.5321693399427, 5.594275661166211]），这个有点不太懂。正常应该是成倍数的？

---> 对比 cv::SolvePnP 和 Bundle Adjustment Guass Newton 结果

=========== cv::SolvePnp ============
R,t 见上面的值
=========== ba gn =============
R= 0.997906 -0.0509194  0.0398875
   0.0498187   0.998362  0.0281209
   -0.041254 -0.0260749   0.998808
t =  -0.126782 -0.00843948   0.0603493
只迭代了 4 次， dx.norm() 就 < 1e-6 了 (收敛退出)
=============
可以看到cv的结果和ba的结果非常接近！
*/
#include <cmath>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <src/visual_odometry/orb/orb.h>

#include "pose.h"

using p3d2d_t = std::pair<std::vector<cv::Point3f>, std::vector<cv::Point2f>>;
using eigen3d_points_t = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using eigen2d_points_t = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

p3d2d_t load_depth_and_make_3d2d_points(const std::string& depth_fpath,
    const std::vector<cv::Point2f>& points2d,
    const cv::Mat& K,
    const std::vector<cv::Point2f>& other_points2d);

void cv_pnp(const std::vector<cv::Point3f>& objects, 
    const std::vector<cv::Point2f>& img_points,
    const cv::Mat& K);

void ba_gauss_newton(const eigen3d_points_t& objects,
    const eigen2d_points_t& img_points,
    const cv::Mat& K);

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "param error.\n"
            << "Usage: " << argv[0] << " img1 img1-depth img2\n";
        return 1; 
    }
    std::vector<std::string> img_paths{argv[1], argv[3]};
    std::vector<cv::Mat> imgs(2);
    std::vector<std::vector<cv::KeyPoint>> kps(2);
    std::vector<cv::Mat> descriptors(2);
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    for (auto i = 0; i < 2; ++i) {
        if (!load_img_and_detect_compute(img_paths.at(i), orb, 
            imgs.at(i), kps.at(i), descriptors.at(i))) {
            return 1;
        }
    }
    std::vector<cv::DMatch> matches{};
    if (match_and_draw(descriptors, imgs, kps, matches)) {
        cv::waitKey(1);
    }
    std::vector<std::vector<cv::Point2f>> match_points = get_match_points(
        kps, matches);

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        520.9, 0, 325.1, 
        0, 521.0, 249.7, 
        0, 0, 1
    );

    std::vector<cv::Point3f> objects{};
    std::vector<cv::Point2f> img_points{};
    std::tie(objects, img_points) = load_depth_and_make_3d2d_points(argv[2], 
        match_points.at(0), 
        K,
        match_points.at(1));
    cv_pnp(objects, img_points, K);
    eigen3d_points_t eigen_objects{};
    eigen_objects.reserve(objects.size());
    eigen2d_points_t eigen_img_points{};
    eigen_img_points.reserve(img_points.size());
    for (std::size_t i = 0U; i < objects.size(); ++i) {
        auto& o = objects.at(i);
        eigen_objects.emplace_back(o.x, o.y, o.z);
        auto& p = img_points.at(i);
        eigen_img_points.emplace_back(p.x, p.y);
    }
    ba_gauss_newton(eigen_objects, eigen_img_points, K);
}

p3d2d_t load_depth_and_make_3d2d_points(const std::string& depth_fpath,
    const std::vector<cv::Point2f>& points2d,
    const cv::Mat& K,
    const std::vector<cv::Point2f>& other_points2d) {
    auto depth_mat = cv::imread(depth_fpath, cv::IMREAD_UNCHANGED);
    std::vector<cv::Point3f> objects{};
    std::vector<cv::Point2f> img_points{};
    objects.reserve(points2d.size());
    img_points.reserve(points2d.size());
    for (std::size_t i = 0U; i < points2d.size(); ++i) {
        auto& p = points2d.at(i);
        auto raw_depth = depth_mat.at<ushort>(p.y, p.x);
        if (raw_depth == 0) {
            // bad
            continue;
        }
        // as slambook
        float actual_depth = raw_depth / 5000.f;
        // 深度，其实是相机空间下的；
        // 所以，x，y也需要转换到相机空间下！
        cv::Point3f camera3d_norm = pixel2camera(p, K);
        cv::Point3f camera3d = camera3d_norm * actual_depth;
        objects.push_back(std::move(camera3d));
        img_points.push_back(other_points2d.at(i));
    }
    return std::make_pair(std::move(objects), std::move(img_points));
}

void cv_pnp(const std::vector<cv::Point3f>& objects, 
    const std::vector<cv::Point2f>& img_points,
    const cv::Mat& K) {
    cv::Mat rvec{};
    cv::Mat tvec{};
    std::clog << "objects num = " << objects.size() 
        << ", img points num = " << img_points.size() << "\n";
    bool is_ok = cv::solvePnP(objects, img_points, K, cv::Mat(), rvec, tvec, false);
    std::clog << "cv::solvePnP is "<< std::boolalpha << is_ok << std::endl;
    std::clog << "rvec = " << rvec.t() << "; tvec = " << tvec.t() << std::endl;
    cv::Mat R{};
    cv::Rodrigues(rvec, R);
    std::clog << "R = " << R << std::endl;
}

void ba_gauss_newton(const eigen3d_points_t& objects,
    const eigen2d_points_t& img_points,
    const cv::Mat& K) {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    constexpr std::size_t max_iterations = 10U;

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    Sophus::SE3d T{};

    double last_cost{};
    for (std::size_t iter = 0U; iter < max_iterations; ++iter) {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d g = Vector6d::Zero();
        double cost{};
        for (auto i = 0U; i < img_points.size(); ++i) {
            auto& object = objects.at(i);
            auto& img_point = img_points.at(i);
            Eigen::Vector3d camera_object = T * object;
            double z = camera_object[2];
            Eigen::Vector3d norm_camera_object = camera_object / z;
            Eigen::Vector2d proj_point(fx * norm_camera_object[0] + cx,
                fy * norm_camera_object[1] + cy);
            // shape = [2, 1]
            Eigen::Vector2d e = img_point - proj_point;
            cost += e.squaredNorm();

            auto inv_z = 1. / z;
            auto inv_z2 = inv_z * inv_z;
            Eigen::Matrix<double, 2, 6> J{};
            J << 
                // row 1
                -fx * inv_z, 0,
                fx * camera_object[0] * inv_z2, fx * camera_object[0] * camera_object[1] * inv_z2,
                -fx - fx * camera_object[0] * camera_object[0] * inv_z2, fx * camera_object[1] * inv_z,
                // row 2
                0, -fy * inv_z,
                fy * camera_object[1] * inv_z2, fy + fy * camera_object[1] * camera_object[1] * inv_z2,
                -fy * camera_object[0] * camera_object[1] * inv_z2, -fy * camera_object[0] * inv_z;
            // matrix multiply. [6, 2] x [2, 6] => [6, 6]
            H += J.transpose() * J;
            // matrix multiply. [6, 2] x [2, 1] => [6, 1]
            g += -J.transpose() * e;
        }
        std::clog << "iter " << iter << " cost = " << cost << "\n";
        if (iter > 0U && cost >= last_cost) {
            std::clog << "bigger/euqal than last cost(" << last_cost << "). break iteration.\n";
            break;
        } 
        Vector6d dx = H.ldlt().solve(g);
        if (std::isnan(dx[0])) {
            std::cerr << "dx is nan\n";
            break;
        }
        // update
        // exp(dx) => a matrix of [4, 4]. x T = [4, 4] => [4, 4]
        T = Sophus::SE3d::exp(dx) * T;
        last_cost = cost;
        if (dx.norm() < 1e-6) {
            std::clog << "dx is too small. converged.\n";
            break;
        }
    }
    Eigen::Matrix<double, 4, 4> T_mat = T.matrix();
    std::clog << "BA by Guass-Newton, T = " << T_mat << "\n"
        << "R = " << T_mat.topLeftCorner<3, 3>() << "\n"
        << "t = " << T_mat.topRightCorner<3, 1>().transpose() << "\n";

}