cmake_minimum_required(VERSION 2.9)
project(orb)

set(CMAKE_CXX_STANDARD 17)

if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -pedantic)
endif()

find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(intensity_centroid intensity_centroid.cpp)
target_link_libraries(intensity_centroid ${OpenCV_LIBS})

add_executable(key_point key_point.cpp)
target_link_libraries(key_point ${OpenCV_LIBS})