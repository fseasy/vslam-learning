cmake_minimum_required(VERSION 2.9)
project( orb )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( intensity_centroid intensity_centroid.cpp )
target_link_libraries( intensity_centroid ${OpenCV_LIBS} )
