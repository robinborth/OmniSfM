#include <iostream>
#include <opencv2/opencv.hpp>

#include "src/SfMOptimizer.h"
#include "src/Eigen.h"
#include "src/VirtualSensor.h"
#include "src/SimpleMesh.h"
#include "src/PointCloud.h"
// #include "src/NearestNeighbor.h"

int main()
{
    std::cout << "Running SfM pipeline" << std::endl;

    // // Read the image
    std::string imagePath = "/Users/robinborth/Code/OmniSfM/data/rgbd_dataset_freiburg1_xyz/rgb/00000.png";
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

    // Check if the image is loaded properly
    if (image.empty())
    {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // Detect keypoints using SIFT
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    sift->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

    // Draw the keypoints on the image
    cv::Mat img_keypoints;
    cv::drawKeypoints(image, keypoints, img_keypoints);

    // Save the result
    cv::imwrite("sift_keypoints.jpg", img_keypoints);

    // // checks that the ann search works
    // std::vector<Vector3f> source_points;
    // source_points.push_back(Vector3f{0.0f, 0.0f, 0.0f});
    // source_points.push_back(Vector3f{1.0f, 0.0f, 0.0f});
    // source_points.push_back(Vector3f{0.0f, 0.0f, 2.0f});

    // std::vector<Vector3f> target_points;
    // target_points.push_back(Vector3f{0.1f, 0.1f, 0.1f});
    // target_points.push_back(Vector3f{2.1f, -0.1f, 0.1f});

    // // Search for matches using FLANN.
    // std::unique_ptr<NearestNeighborSearch> nearestNeighborSearch = std::make_unique<NearestNeighborSearchFlann>();
    // nearestNeighborSearch->setMatchingMaxDistance(0.0001f);
    // nearestNeighborSearch->buildIndex(target_points);
    // auto matches = nearestNeighborSearch->queryMatches(source_points);

    return 0;
}
