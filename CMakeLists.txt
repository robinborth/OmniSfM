cmake_minimum_required(VERSION 3.15)

project(sfm)

# options
set(CMAKE_CXX_STANDARD 17)
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(glog_DIR ${LIBRARY_DIR}/glog-lib/lib/cmake/glog CACHE PATH "Path to installed glog")
set(Ceres_DIR ${LIBRARY_DIR}/Ceres/lib/cmake/Ceres CACHE PATH "Path to installed Ceres")
set(Flann_INCLUDE_DIR ${LIBRARY_DIR}/Flann/include CACHE PATH "Path to Flann source folder")

# freeimage mac m1 setup
set(FREE_IMAGE_DIR /opt/homebrew/Cellar/freeimage/3.18.0)  
include_directories(${FREE_IMAGE_DIR}/include/)
link_directories(${FREE_IMAGE_DIR}/lib/)

# find the packages 
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)


add_executable(sfm main.cpp)
target_link_libraries(sfm ceres Eigen3::Eigen)
target_include_directories(sfm PUBLIC ${EIGEN3_INCLUDE_DIR} ${Flann_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src)