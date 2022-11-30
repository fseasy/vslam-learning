#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>
/*
 单目的地图构建

给定：
    1. 连续时间的图片（关键帧）
    2. 对应的相机位姿

目标：
    1. 计算出第一张图片里的每个像素的深度值
    2. 可视化为点云

方法：
    深度滤波
    1. 选择目标图片（第一张）为基准图片；预设每个像素深度符合高斯分布，且均值为 m, 方差为 u
    2. 依次选择后续的图片i，对（图片i，基准图片）
        a. 基准图片每个点，找到图片i对应的点（块匹配，极线搜索 + NCC）
        b. 对匹配像素点，三角化计算出深度 dm
        c. 按照高斯分布合并方法，更新像素深度，方差
*/


int main(int argc, char* argv[]) {



    return 0;
}