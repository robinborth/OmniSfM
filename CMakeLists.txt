cmake_minimum_required(VERSION 3.15)

project(sfm)

# options
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(glog_DIR ${LIBRARY_DIR}/glog-lib/lib/cmake/glog CACHE PATH "Path to installed glog")
set(Ceres_DIR ${LIBRARY_DIR}/Ceres/lib/cmake/Ceres CACHE PATH "Path to installed Ceres")

# find the packages 
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)


add_executable(sfm main.cpp)
target_link_libraries(sfm ceres Eigen3::Eigen)
target_include_directories(sfm PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src)