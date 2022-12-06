#pragma once

#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <optional>

#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <fmt/core.h>

namespace dataset {

class Dataset {
public:
    explicit Dataset(const std::string& data_dir, 
        std::size_t process_num=std::numeric_limits<std::size_t>::max());

    std::size_t size() const noexcept { return img_fpaths_.size(); }
    
    std::optional<cv::Mat> get_img(std::size_t index) const;
    std::optional<Sophus::SE3d> get_pose(std::size_t index) const;
    std::optional<cv::Mat> get_ref_depth(std::size_t index) const;

private:
    const std::string data_dir_;
    std::vector<std::string> img_fpaths_;
    std::vector<Sophus::SE3d> poses_; 
};

// inline impl

inline
Dataset::Dataset(const std::string& data_dir, std::size_t process_num) 
    : data_dir_(data_dir) {
    const std::string pose_fname = "/first_200_frames_traj_over_table_input_sequence.txt";
    auto pose_fpath = data_dir + pose_fname;
    std::ifstream pose_file(pose_fpath);
    if (!pose_file) {
        throw std::invalid_argument("pose path not exist: " + pose_fpath);
    }

    std::size_t line_cnt = 0U;
    const std::string img_dir = data_dir + "/images/"; // with "/" trails for efficient
    while (pose_file) {
        if (line_cnt >= process_num) {
            break;
        }
        std::string img_fname{};
        pose_file >> img_fname;
        double t0, t1, t2;
        pose_file >> t0 >> t1 >> t2;
        double x, y, z, w;
        pose_file >> x >> y >> z >> w;
        Sophus::SE3d pose(Eigen::Quaterniond(w, x, y, z), Eigen::Vector3d(t0, t1, t2));
        img_fpaths_.push_back(img_dir + img_fname);
        poses_.push_back(std::move(pose));
        ++line_cnt;
    }
    pose_file.close();
}

inline 
std::optional<cv::Mat> Dataset::get_img(std::size_t index) const {
    if (index >= size()) {
        return {};
    }  
    auto& fpath = img_fpaths_.at(index);
    std::optional<cv::Mat> img = cv::imread(fpath, cv::IMREAD_GRAYSCALE);
    return img;
}

inline
std::optional<Sophus::SE3d> Dataset::get_pose(std::size_t index) const {
    if (index >= size()) {
        return {};
    }
    std::optional<Sophus::SE3d> twc = poses_.at(index);
    return twc;
}

inline
std::optional<cv::Mat> Dataset::get_ref_depth(std::size_t index) const {
    if (index >= size()) {
        return {};
    }
    const std::string depth_fpath = fmt::format(
        "{}/depthmaps/scene_{:03d}.depth",
        data_dir_, index);
    auto depth = cv::imread(depth_fpath, cv::IMREAD_UNCHANGED);
    std::clog << "readed depth type = " << depth.type() << "\n";
    depth /= 100.;
    return depth;
}

} // end of dataset

