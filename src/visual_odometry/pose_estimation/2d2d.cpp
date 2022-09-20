#include <iostream>
#include <random>

#include <opencv2/opencv.hpp>
#include <src/visual_odometry/orb/orb.h>

std::vector<std::vector<cv::Point2f>> _get_match_points(
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    const std::vector<cv::DMatch>& matches);

void epipolar_geometry(
    const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, 
    cv::Mat& t,
    cv::Mat& E);

void verify_epipolar(
    const std::vector<std::vector<cv::Point2f>>& match_points,
    const cv::Mat& camera_intrinsic,
    const cv::Mat& R,
    const cv::Mat& t,
    const cv::Mat& E);

void verify_F_and_draw_epilines(
    const std::vector<std::vector<cv::Point2f>>& match_points,
    const cv::Mat& camera_intrinsic,
    const cv::Mat& E,
    const std::vector<cv::Mat>& imgs
);

void homography(
    const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, 
    cv::Mat& t);

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "param error.\n"
            << "Usage: " << argv[0] << " img1 img2\n";
        return 1; 
    }
    const std::vector<std::string> img_paths{argv[1], argv[2]};
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

    auto match_points = _get_match_points(kps, matches);
    cv::Mat camera_intrinsic = (cv::Mat_<double>(3, 3) 
        << 520.9, 0, 325.1, 
           0, 521.0, 249.7, 
           0, 0, 1
        );
    cv::Mat R{};
    cv::Mat t{};
    cv::Mat E{};
    epipolar_geometry(match_points, camera_intrinsic, R, t, E);
    verify_epipolar(match_points, camera_intrinsic, R, t, E);
    verify_F_and_draw_epilines(match_points, camera_intrinsic, E, imgs);
    homography(match_points, camera_intrinsic, R, t);
}

std::vector<std::vector<cv::Point2f>> _get_match_points(
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

void epipolar_geometry(
    const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, 
    cv::Mat& t,
    cv::Mat& E) {
    std::clog << "Epipolar Geometry get match point pairs " << match_points.at(0).size() 
        << std::endl;
    AutoTimer timer("epipolar geometry");
    // for new version, may be try: cv::USAC_MAGSAC, see
    // https://opencv.org/evaluating-opencvs-new-ransacs/
    cv::Mat outlier_indicator{};
    E = cv::findEssentialMat(match_points.at(0), match_points.at(1), 
        camera_intrinsic, cv::RANSAC, 
        0.999, 1.0, outlier_indicator);
    timer.duration_ms("find-E");
    cv::Mat recover_outliter_indicator = outlier_indicator.clone();
    // 
    // Cheirality check: triangulated 3D points should have positive depth
    // 内部步骤可以参考 OpenCV api文档，或者 https://stackoverflow.com/questions/65771642/opencv-recoverpose-from-essential-matrix-e
    int cheirality_check_pnt_num = cv::recoverPose(E, 
        match_points.at(0), match_points.at(1), camera_intrinsic,
        R, t, recover_outliter_indicator);
    timer.duration_ms("decompose-E");
    cv::Mat rvec{};
    cv::Rodrigues(R, rvec);
    std::clog << "E = \n" << E << "\n";
    std::clog << "in find-E, outlier num = " << cv::sum(1 - outlier_indicator) << "\n";
    std::clog << "R = \n" << R << "\n";
    std::clog << "t = " << t.t() << "\n";
    std::clog << "Rodrigues = " << rvec.t() << "\n";
    cv::Mat r_norm(2, 3, CV_32FC1);
    for (auto i = 0; i < 3; ++i) {
        r_norm.at<float>(0, i, 0) = cv::norm(R.row(i));
        r_norm.at<float>(1, i, 0) = cv::norm(R.col(i));
    }
    auto t_norm = cv::norm(t);
    std::clog << "R norm2 = " << r_norm << " " << "t norm2 = " << t_norm << "\n";
    std::clog << "in recover-Pose, outlier num = " << cv::sum(1 - recover_outliter_indicator) 
        << "\n"
        << "cheirality check cnt = " << cheirality_check_pnt_num << "\n";
}

void verify_epipolar(
    const std::vector<std::vector<cv::Point2f>>& match_points,
    const cv::Mat& camera_intrinsic,
    const cv::Mat& R,
    const cv::Mat& t,
    const cv::Mat& E) {
    // 跟前面 RANSAC 计算的E不一样！ 试了 slambook2 代码的结果，和这里结果一样。
    // it may because we abandon the minimum eigenvalue in SVD
    // 如何衡量两个矩阵间的误差呢？这是个问题。
    // 1. E = t^R
    // t_x = 
    // | 0   | -a3 | a2 |
    // | a3  | 0   | -a1|
    // | -a2 | a1  | 0  |
    const cv::Mat_<double>& t_ = static_cast<cv::Mat_<double>>(t);
    cv::Mat t_x = (cv::Mat_<double>(3, 3) <<
        0, -t_(2, 0), t_(1, 0),
        t_(2, 0), 0, -t_(0, 0),
        -t_(1, 0), t_(0, 0), 0
    );
    cv::Mat E_rebuild = t_x * R;
    std::clog << "E = t^R = " << E_rebuild << "\n"; 
    std::clog << "original E = " << E << "\n";

    auto _pixel_pnt2camera_3d = [&camera_intrinsic](const cv::Point2f& p) -> cv::Mat {
        auto K_ = static_cast<cv::Mat_<double>>(camera_intrinsic);
        double cx = K_(0, 2);
        double cy = K_(1, 2);
        double fx = K_(0, 0);
        double fy = K_(1, 1);
        return (cv::Mat_<double>(3, 1) << 
            (p.x - cx) / fx,
            (p.y - cy) / fy,
            1
        );
    };
    auto _epipolar_constraint = [](const auto& p1, const auto& p2, const auto& E) {
        cv::Mat constraint = p2.t() * E * p1;
        return constraint.at<double>(0, 0);
    };

    for (std::size_t i = 0U; i < match_points.at(0).size(); ++i) {
        auto p1 = _pixel_pnt2camera_3d(match_points.at(0).at(i));
        auto p2 = _pixel_pnt2camera_3d(match_points.at(1).at(i));
        double constraint_rebuild = _epipolar_constraint(p1, p2, E_rebuild);
        double constraint_original = _epipolar_constraint(p1, p2, E);
        std::clog << "point pair " << i << ", rebuild epipolar constraint = " << constraint_rebuild
            << ", original constraint = " << constraint_original << "\n";
    }
}

void verify_F_and_draw_epilines(
    const std::vector<std::vector<cv::Point2f>>& match_points,
    const cv::Mat& camera_intrinsic,
    const cv::Mat& E,
    const std::vector<cv::Mat>& imgs) {
    // 1. calc from findFundamentalMat
    cv::Mat mask{};
    cv::Mat F_calc = cv::findFundamentalMat(match_points.at(0), match_points.at(1), 
        cv::FM_RANSAC, 3., 0.99, mask);
    std::cerr << "F(findFundamentalMat) = " << F_calc << "\n";
    // 2. calc from K^{-T} E K^{-1}
    cv::Mat K_inv = camera_intrinsic.inv();
    cv::Mat F_from_E = K_inv.t() * E * K_inv;
    std::cerr << "F(K^{-T}EK^{-1}) = " << F_from_E << "\n"; 
    // norm F[2][2] = 1
    auto times = 1. / F_from_E.at<double>(2, 2);
    F_from_E *= times;
    std::cerr << "F(K^{-T}EK^{-1} x norm) = " << F_from_E << "\n";
    // 同样是不等的…… 目前还不知道如何去衡量两个 F 的差距
    
    // 下面画极线。 
    // 图1上的点，对应的极线，是在图2上的！反之亦然。
    cv::Mat epilines_for_pnt1{};
    cv::computeCorrespondEpilines(match_points.at(0), 1, F_calc, 
        epilines_for_pnt1);
    cv::Mat epilines_for_pnt2{};
    cv::computeCorrespondEpilines(match_points.at(1), 2, F_calc, 
        epilines_for_pnt2);
    
    std::clog << "epilines size = " << epilines_for_pnt1.size() 
        << " type = " << epilines_for_pnt1.type() << "\n";

    std::mt19937 _gen(12304U);
    std::uniform_int_distribution<uchar> _dist(0, 255);
    auto uchar_dist = std::bind(_dist, _gen);
    auto draw_epilines = [&uchar_dist](const cv::Mat& epilines,
        auto& target_img, auto& source_img,
        const auto& pnts_in_target_img, const auto& pnts_in_source_img,
        const cv::Mat& mask
        ) {
        // epilines: ax + by + c = 0, know a, b, c. draw line.
        // as https://docs.opencv.org/4.x/da/de9/tutorial_py_epipolar_geometry.html
        // use 2 points
        // 1. let x = 0; y = -c / b
        // 2. let x = img.col, y = - (c + a x) / b
        // because we need the line full cross the img.
        // so let x = 0, col is a easy way. or let y = 0, row is another.
        // 取 x = 0 and y = 0 对应的两个点，不能达成线完全覆盖图片的目的！
        auto pnt_sz = pnts_in_target_img.size();
        for (auto i = 0U; i < pnt_sz; ++i) {
            uchar pnt_mask = mask.at<uchar>(i);
            if (pnt_mask == 0) {
                // outlier
                std::clog << "Point " << i << " is outlier, skipped\n";
                continue;
            }
            // 只画20%的点，太多看不过来
            if (uchar_dist() / 256.f < 0.8) {
                continue;
            }
            cv::Scalar color(uchar_dist(), uchar_dist(), uchar_dist());
            // std::clog << "color = " << color << "\n";
            auto& epiline = epilines.at<cv::Vec3f>(i);
            int x1 = 0;
            int y1 = - epiline(2) / epiline(1);
            int x2 = target_img.cols;
            int y2 = - (epiline(2) + epiline(0) * x2) / epiline(1);
            cv::Point p1{x1, y1};
            cv::Point p2{x2, y2};
            cv::line(target_img, p1, p2, color, 1, cv::LINE_AA, 0);
            // draw correspond points in 2 img. target use circle, source use cross
            auto& target_p = pnts_in_target_img.at(i);
            cv::circle(target_img, target_p, 5, color, -1, cv::LINE_AA);
            auto& source_p = pnts_in_source_img.at(i);
            cv::drawMarker(source_img, source_p, color, cv::MARKER_CROSS, 20, 
                2, cv::LINE_AA);
        }
    };
    cv::Mat draw_img1 = imgs.at(0).clone();
    cv::Mat draw_img2 = imgs.at(1).clone();
    draw_epilines(epilines_for_pnt1, draw_img2, draw_img1, 
        match_points.at(1), match_points.at(0), mask);
    cv::Mat cat_img{};
    cv::hconcat(draw_img1, draw_img2, cat_img);
    cv::putText(cat_img, 
        "Left: feature points "
        "=> Right: coresponding epilines & feature points",
        cv::Point2i(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1,
        cv::Scalar::all(255), 2, cv::LINE_AA);
    cv::imshow("epilines: img1->img2", cat_img);
    cv::imwrite("epilines.png", cat_img);
    cv::waitKey(0);
}


void homography(
    const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, 
    cv::Mat& t) {
    std::clog << "Homography get match points size = " << match_points.at(0).size() << "\n";
    AutoTimer timer("homography");
    cv::Mat H = cv::findHomography(match_points.at(0), match_points.at(1), 
        cv::RANSAC, 3);
    timer.duration_ms("find-homography");
    // method1: use decomose-homography
    // see: https://docs.opencv.org/3.4/de/d45/samples_2cpp_2tutorial_code_2features2D_2Homography_2decompose_homography_8cpp-example.html#a22
    std::vector<cv::Mat> Rs{};
    std::vector<cv::Mat> ts{};
    std::vector<cv::Mat> normals{};
    int solutions = cv::decomposeHomographyMat(H, camera_intrinsic, Rs, ts, normals);
    timer.duration_ms("decompose-homography");
    std::clog << "H = " << H << "\n";
    std::clog << "solutions = " << solutions << "\n";
    std::clog << Rs.size() << "\n";
    cv::Mat rvec{};
    for (int i = 0; i < solutions; ++i) {
        std::clog << "===> solution " << i + 1 << "\n";
        std::clog << "R = " << Rs.at(i) << " t = " << ts.at(i).t() 
            << " normal = " << normals.at(i).t() << "\n";
        cv::Rodrigues(Rs.at(i), rvec);
        std::clog << "Rodrigues: " << rvec.t() << "\n";
    }
    // method2: use hand-recover
    // see: https://docs.opencv.org/3.4/d0/d92/samples_2cpp_2tutorial_code_2features2D_2Homography_2pose_from_homography_8cpp-example.html#a16
    double norm = cv::norm(H.col(0));
    H /= norm;
    cv::Mat c1 = H.col(0);
    cv::Mat c2 = H.col(1);
    cv::Mat c3 = c1.cross(c2);

    t = H.col(2);
    R = cv::Mat(3, 3, CV_64F);
    c1.copyTo(R.col(0));
    c2.copyTo(R.col(1));
    c3.copyTo(R.col(2));
    std::clog << "R(before polar decomposition):\n" << R;
    double Rdet = cv::determinant(R);
    std::clog << " determinant = " << Rdet << "\n";
    cv::Rodrigues(R, rvec);
    std::clog << "Rodrigues = " << rvec.t() << "\n";
    cv::Mat W{};
    cv::Mat U{};
    cv::Mat Vt{};
    cv::SVDecomp(R, W, U, Vt);
    R = U * Vt;
    Rdet = cv::determinant(R);
    if (Rdet < 0) {
        Vt.row(2) = Vt.row(2) * -1;
        R = U * Vt;
        Rdet = cv::determinant(R);
        std::clog << "determinant is negative. re calc = " << Rdet << "\n";
    }
    std::clog << "R(after polar decomposition):\n" << R << " determinant = " << Rdet << "\n";
    std::clog << "t = " << t.t() << "\n";
    cv::Rodrigues(R, rvec);
    std::clog << "Rodrigues = " << rvec.t() << "\n";
    // 从结果来看，
    // 1. 调用 decomposeH 的结果的四组里，第一组结果和前面对极集合的结果很接近！
    // 2. 而下面这种手工做法，结果和上面方法的结果都差挺远的。感觉不太靠谱。
}