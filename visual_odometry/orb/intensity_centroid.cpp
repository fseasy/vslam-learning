#include <iostream>

#include <opencv2/opencv.hpp>

bool CalcIntensityCentroid(const cv::Mat& img,
    const cv::Point2f& patch_center, 
    std::size_t patch_size,
    cv::Point2f& centroid_point) {
    // corner check
    int patch_half_size = static_cast<int>(patch_size / 2);
    // ---c-- (patch_size is odd, )
    if (patch_center.x - patch_half_size < 0
        || patch_center.y - patch_half_size < 0
        || patch_center.x + patch_half_size - 1 >= img.cols
        || patch_center.y + patch_half_size - 1 >= img.rows) {
        return false;
    }
    float m00{};
    float m01{};
    float m10{};
    for (int dy = -patch_half_size; dy < patch_half_size; ++dy) {
        for (int dx = -patch_half_size; dx < patch_half_size; ++dx) {
            int ix = patch_center.x + dx;
            int iy = patch_center.y + dy;
            // `uchar` is defined in OpenCV and in global name space!!
            uchar intensity = img.at<uchar>(iy, ix);
            m00 += intensity;
            m01 += dy * intensity;
            m10 += dx * intensity;
        }
    }
    float patch_centroid_x = static_cast<float>(m10) / m00;
    float patch_centroid_y = static_cast<float>(m01) / m00;
    centroid_point.x = patch_centroid_x + patch_center.x;
    centroid_point.y = patch_centroid_y + patch_center.y;
    return true;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " img-path" << std::endl;
        return -1;
    }
    auto img_fpath = argv[1];
    cv::Mat img = cv::imread(img_fpath, cv::IMREAD_GRAYSCALE);
    if (!img.data) {
        std::cerr << "img path '" << img_fpath << "' is not a valid gray image." 
            << std::endl;
        return -1;
    }
    cv::imshow("input-image-show", img);
    cv::waitKey(5000);
    cv::Point2f center_point(30.f, 30.f);
    cv::Point2f centroid_point{};
    bool is_ok = CalcIntensityCentroid(img, center_point, 16U, centroid_point);
    std::cerr << "calc intensity centroid " << std::boolalpha << is_ok << std::endl;
    std::cerr << "centroid = " << centroid_point << std::endl;
    return 0;
}