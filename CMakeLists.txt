cmake_minimum_required(VERSION 3.6)

project(sfm)

# options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(glog_DIR ${LIBRARY_DIR}/glog-lib/lib/cmake/glog CACHE PATH "Path to installed glog")
set(Ceres_DIR ${LIBRARY_DIR}/Ceres/lib/cmake/Ceres CACHE PATH "Path to installed Ceres")
# set(Flann_INCLUDE_DIR ${LIBRARY_DIR}/Flann/include CACHE PATH "Path to Flann source folder")
set(OpenCV_DIR ${LIBRARY_DIR}/opencv/lib/cmake/opencv4 CACHE PATH "Path to OpenCV source folder")

# freeimage mac m1 setup
set(FREE_IMAGE_DIR /opt/homebrew/Cellar/freeimage/3.18.0)  
include_directories(${FREE_IMAGE_DIR}/include/)
link_directories(${FREE_IMAGE_DIR}/lib/)

# set(LZ4_DIR /opt/homebrew/Cellar/lz4/1.9.4)  
# include_directories(${LZ4_DIR}/include/)
# link_directories(${LZ4_DIR}/lib/)

# Set C++ flags
# todo make opencv 14
set(CMAKE_CXX_STANDARD 14)

# find the packages 
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
get_target_property(GLOG_DLL_PATH_DEBUG glog::glog IMPORTED_LOCATION_DEBUG)
get_target_property(GLOG_DLL_PATH_RELEASE glog::glog IMPORTED_LOCATION_RELEASE)

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

add_executable(sfm main.cpp)
target_link_libraries(sfm ceres Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(sfm PUBLIC ${EIGEN3_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src)
# target_include_directories(sfm PUBLIC ${EIGEN3_INCLUDE_DIR} ${Flann_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/src)
# target_link_libraries(sfm ceres Eigen3::Eigen)
# target_include_directories(sfm PUBLIC ${EIGEN3_INCLUDE_DIR} ${Flann_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/src)