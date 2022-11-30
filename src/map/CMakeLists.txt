cmake_minimum_required(VERSION 2.9)

project(map)

set(CMAKE_CXX_STANDARD 17)
if (MSVC)
    add_compile_options(/W /WX)
else()
    add_compile_options(-Wall -Wextra -pedantic -ansi)
endif()

set(WORK_ROOT "../../")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

message(STATUS "eigen3 include dir =" ${Eigen3_INCLUDE_DIRS})
message(STATUS "Sophus include dir =" ${Sophus_INCLUDE_DIRS})
message(STATUS "OpenCV include dir =" ${OpenCV_INCLUDE_DIRS})

add_executable(monocolar_dense monocular_dense.cpp)
target_link_libraries(monocolar_dense 
    ${OpenCV_LIBS} 
    Eigen3::Eigen 
    Sophus::Sophus)
target_include_directories(monocolar_dense PRIVATE ${OpenCV_INCLUDE_DIRS})