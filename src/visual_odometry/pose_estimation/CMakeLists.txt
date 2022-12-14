
cmake_minimum_required(VERSION 2.9)
project(pose-estimation)

set(CMAKE_CXX_STANDARD 17)
if(MSVC)
    add_compile_options(/W /WX)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

SET(WORK_ROOT "../../..")

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

message(STATUS "eigen3 include dir =" ${Eigen3_INCLUDE_DIRS})
message(STATUS "Sophus include dir =" ${Sophus_INCLUDE_DIRS})
message(STATUS "OpenCV include dir =" ${OpenCV_INCLUDE_DIRS})

# add_executable(2d2d 2d2d.cpp)
# target_link_libraries(2d2d ${OpenCV_LIBS})
# target_include_directories(2d2d PRIVATE ${WORK_ROOT})

# add_executable(triangulate triangulate.cpp)
# target_link_libraries(triangulate ${OpenCV_LIBS})
# target_include_directories(triangulate PRIVATE ${WORK_ROOT})

add_executable(3d2d 3d2d.cpp)
target_link_libraries(3d2d ${OpenCV_LIBS} Eigen3::Eigen Sophus::Sophus)
target_include_directories(3d2d PRIVATE ${WORK_ROOT})