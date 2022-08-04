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
    std::size_t m00 = 0U;
    std::size_t m01 = 0U;
    std::size_t m10 = 0U;
    for (int dy = -patch_half_size; dy < patch_half_size; ++dy) {
        for (int dx = -patch_half_size; dx < patch_half_size; ++dx) {
            int ix = patch_center.x + dx;
            int iy = patch_center.y + dy;
            // uchar is defined in OpenCV and in global name space!!
            uchar intensity = img.at<uchar>(iy, ix);

        }
    }

}

int main() {
    return 0;
}