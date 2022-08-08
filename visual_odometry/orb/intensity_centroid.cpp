#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>

bool CalcIntensityCentroid(const cv::Mat& img,
    const cv::Point2f& patch_center, 
    int patch_size,
    cv::Point2f& centroid_point) {
    // corner check
    int patch_half_size = static_cast<int>(patch_size / 2);
    //  patch_size is odd, and center in right, like: ---|c--
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
    float patch_centroid_x = m10 / m00;
    float patch_centroid_y = m01 / m00;
    centroid_point.x = patch_centroid_x + patch_center.x;
    centroid_point.y = patch_centroid_y + patch_center.y;
    return true;
}

void drawIntensityCentroid(const cv::Mat& img, 
    const cv::Point2f& center, 
    const cv::Point2f& centroid,
    int patch_size);

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
    int width = img.cols;
    int height = img.rows;
    cv::Point2f center_point(width / 2, height / 2);
    int patch_size = 300;
    cv::Point2f centroid_point{};
    bool is_ok = CalcIntensityCentroid(img, center_point, patch_size, centroid_point);
    if (!is_ok) {
        std::cerr << "Calc intensity centroid failed." << std::endl;
        return -1;
    }
    std::cerr << "Center = " << center_point 
        << ", Intensity centroid = " << centroid_point << std::endl;
    
    float theta = atan2f(centroid_point.y - center_point.y, 
        centroid_point.x - center_point.x);
    std::cerr << "theta = " << theta << ", " << theta / M_PI * 180 << std::endl;
    drawIntensityCentroid(img, center_point, centroid_point, patch_size);
    return 0;
}

void drawIntensityCentroid(const cv::Mat& img, 
    const cv::Point2f& center, 
    const cv::Point2f& centroid,
    int patch_size) {
    cv::Mat img_draw{};
    cv::cvtColor(img, img_draw, cv::COLOR_GRAY2BGR);

    // coordinates
    const int thickness = 1;
    cv::Point2f y_axis_start_pnt(center.x, img.rows * 0.1f);
    cv::Point2f y_axis_end_pnt(center.x, img.rows * 0.9f);
    cv::Point2f x_axis_start_pnt(img.cols * 0.1f, center.y);
    cv::Point2f x_axis_end_pnt(img.cols * 0.9f, center.y);
    cv::arrowedLine(img_draw, y_axis_start_pnt, y_axis_end_pnt, CV_RGB(255, 0, 0), 
        thickness, cv::LINE_AA, 0, 0.05);
    cv::arrowedLine(img_draw, x_axis_start_pnt, x_axis_end_pnt, CV_RGB(255, 0, 0),
        thickness, cv::LINE_AA, 0, 0.05);
    const int fontface = cv::FONT_HERSHEY_COMPLEX_SMALL;
    const int font_height = 16;
    double fontscale = cv::getFontScaleFromHeight(fontface, font_height, thickness);
    cv::Point2f y_axis_tag_pnt = y_axis_end_pnt + 
        cv::Point2f(10, 0);
    cv::putText(img_draw, "y", y_axis_tag_pnt, 
        fontface, fontscale, cv::Scalar::all(0), thickness);
    cv::Point2f x_axis_tag_pnt = x_axis_end_pnt + 
        cv::Point2f(-10, 20);
    cv::putText(img_draw, "x", x_axis_tag_pnt, 
        fontface, fontscale, cv::Scalar::all(0), thickness);
    // patch range
    int half_patch_size = patch_size / 2;
    cv::Point2f patch_left_up_pnt = center + 
        cv::Point2f(-half_patch_size, -half_patch_size);
    cv::Point2f patch_right_bottom_pnt = center +
        cv::Point2f(half_patch_size - 1, half_patch_size - 1);
    cv::rectangle(img_draw, patch_left_up_pnt, patch_right_bottom_pnt, 
        CV_RGB(0, 255, 0), 1, cv::LINE_AA);
    // center, centroid
    cv::drawMarker(img_draw, center, CV_RGB(160, 32, 240), cv::MARKER_TILTED_CROSS, 10, 
        1, cv::LINE_AA);
    cv::drawMarker(img_draw, centroid, CV_RGB(240, 215, 0), cv::MARKER_TRIANGLE_UP, 10, 
        1, cv::LINE_AA);
    // theta
    float dy = centroid.y - center.y;
    float dx = centroid.x - center.x;
    float hypotenuse = std::hypotf(dy, dx);
    const float theta_line_radius = center.x;
    cv::Point2f theta_end_pnt = cv::Point2f(
        theta_line_radius * dx / hypotenuse, theta_line_radius * dy / hypotenuse
    ) + center;
    cv::line(img_draw, center, theta_end_pnt, CV_RGB(240, 215, 0), 1, cv::LINE_AA);
    cv::imshow("centroid-image", img_draw);
    cv::waitKey(0);
}