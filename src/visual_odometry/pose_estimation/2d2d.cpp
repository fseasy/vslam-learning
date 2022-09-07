#include <iostream>

#include <opencv2/opencv.hpp>
#include <src/visual_odometry/orb/orb.h>

std::vector<std::vector<cv::Point2f>> _get_match_points(
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    const std::vector<cv::DMatch>& matches);

void epipolar_geometry(
    const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, 
    cv::Mat& t);

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
        cv::waitKey(50000);
    }

    auto match_points = _get_match_points(kps, matches);
    cv::Mat camera_intrinsic = (cv::Mat_<double>(3, 3) 
        << 520.9, 0, 325.1, 
           0, 521.0, 249.7, 
           0, 0, 1
        );
    cv::Mat R{};
    cv::Mat t{};
    epipolar_geometry(match_points, camera_intrinsic, R, t);
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
    cv::Mat& t) {
    std::clog << "Epipolar Geometry get match point pairs " << match_points.size() 
        << std::endl;
    AutoTimer timer("epipolar geometry");
    // for new version, may be try: cv::USAC_MAGSAC, see
    // https://opencv.org/evaluating-opencvs-new-ransacs/
    cv::Mat outlier_indicator{};
    cv::Mat E = cv::findEssentialMat(match_points.at(0), match_points.at(1), 
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

void homography(
    const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, 
    cv::Mat& t) {
    std::clog << "Homography get match points size = " << match_points.size() << "\n";
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