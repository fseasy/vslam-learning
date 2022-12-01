#pragma once

#include <string>
#include <vector>
#include <limits>
#include <fstream>

#include <sophus/se3.hpp>
#include <Eigen/Core>


class Dataset {
public:
    explicit Dataset(const std::string& data_dir, 
        std::size_t process_num=std::numeric_limits<std::size_t>::max());

private:
    std::vector<std::string> img_files{};
    std::vector<Sophus::SE3d> poses{}; 
};

explicit Dataset::Dataset(const std::string& data_dir, std::size_t process_num) {
    const std::string pose_fname = "/first_200_frames_traj_over_table_input_sequence.txt";
    auto pose_fpath = data_dir + pose_fname;
    std::ifstream pose_file{pose_fpath};
    if (!pose_file) {
        throw std::invalid_argument("pose path not exist: " + pose_fpath);
    }

    while (pose_file) {

    }
}