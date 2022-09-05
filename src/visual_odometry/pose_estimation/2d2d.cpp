#include <iostream>

#include <opencv2/opencv.hpp>
#include <src/visual_odometry/orb/orb.h>

std::vector<std::vector<cv::Point>> _get_match_points(
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    const std::vector<cv::DMatch>& matches);
void eipolar_geometry(const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, cv::Mat& t);

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
    auto kp2pnts_fn = [](const auto& kps, const ) {
        std::vector<cv::Point2f> pnts{};
        pnts.reserve(kps.size());
        std::transform(kps.cbegin(), kps.cend(), std::back_inserter(pnts), 
            [](const auto& kp){ return kp.pt; });
        return pnts;
    };
    std::vector<std::vector<cv::Point2f>> pnts{
        kp2pnts_fn(kps.at(0)),
        kp2pnts_fn(kps.at(1))
    };

    cv::Mat camera_intrinsic = (cv::Mat_<double>(3, 3) 
        << 520.9, 0, 325.1, 
           0, 521.0, 249.7, 
           0, 0, 1
        );
    cv::Mat R{};
    cv::Mat t{};
    epipopar_geometry(pnts, matches, camera_intrinsic, R, t);
}

std::vector<std::vector<cv::Point>> _get_match_points(
    const std::vector<std::vector<cv::KeyPoint>>& kps,
    const std::vector<cv::DMatch>& matches) {
    
}

void eipolar_geometry(const std::vector<std::vector<cv::Point2f>>& match_points, 
    const cv::Mat& camera_intrinsic,
    cv::Mat& R, cv::Mat& t) {
    
}