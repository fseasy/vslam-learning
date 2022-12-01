#pragma once

#include <string>
#include <vector>
#include <limits>

class Dataset {
public:
    explicit Dataset(const std::string& data_dir, 
        std::size_t process_num=std::numeric_limits<std::size_t>::max());

private:
    std::vector<std::string> img_files{};

};