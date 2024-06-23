cmake_minimum_required(VERSION 3.6)

project(sfm)

# options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/eigen/share/eigen3/cmake CACHE PATH "Path to installed Eigen")
set(glog_DIR ${LIBRARY_DIR}/glog/lib/cmake/glog CACHE PATH "Path to installed glog")
set(Ceres_DIR ${LIBRARY_DIR}/ceres/lib/cmake/Ceres CACHE PATH "Path to installed Ceres")
set(OpenCV_DIR ${LIBRARY_DIR}/opencv/lib/cmake/opencv4 CACHE PATH "Path to OpenCV source folder")
set(Flann_INCLUDE_DIR ${LIBRARY_DIR}/flann/include CACHE PATH "Path to Flann source folder")

# freeimage mac m1 setup
set(FREE_IMAGE_DIR /opt/homebrew/Cellar/freeimage/3.18.0)  
include_directories(${FREE_IMAGE_DIR}/include/)
link_directories(${FREE_IMAGE_DIR}/lib/)

# Set C++ flags
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find the packages 
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
get_target_property(GLOG_DLL_PATH_DEBUG glog::glog IMPORTED_LOCATION_DEBUG)
get_target_property(GLOG_DLL_PATH_RELEASE glog::glog IMPORTED_LOCATION_RELEASE)

add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

# Collect source files
file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Create executable
add_executable(sfm main.cpp ${SRC_FILES})

# Link libraries
target_link_libraries(sfm ceres Eigen3::Eigen ${OpenCV_LIBS})

# Set include directories for the target
target_include_directories(sfm PUBLIC ${EIGEN3_INCLUDE_DIR} ${Flann_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

# set the root directory 
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\")