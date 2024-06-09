#include <iostream>
#include <opencv2/opencv.hpp>

#include "src/Eigen.h"
#include "src/ImageStorage.h"
#include "src/SfMOptimizer.h"
#include "src/PointCloud.h"
#include "src/NearestNeighbor.h"

int main()
{
    std::string dataDir = "/Users/robinborth/Code/OmniSfM/data/rgbd_dataset_freiburg1_xyz/";
    std::cout << "==> Creating image store ..." << std::endl;
    ImageStorage imageStorage(dataDir);
    std::cout << "==> Load images ..." << std::endl;
    imageStorage.loadImages();
    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();
    imageStorage.drawKeypoints(0, "keypoints.jpg");

    std::cout << "(TODO) ==> Find correspondences ..." << std::endl;

    std::cout << "(TODO) ==> Optimize SfM ..." << std::endl;

    std::cout << "(TODO) ==> Create dense point cloud ..." << std::endl;

    std::cout << "(TODO) ==> Create mesh ..." << std::endl;

    std::cout << "(TODO) ==> Save mesh ..." << std::endl;

    // checks that the ann search works
    std::vector<Vector3f> source_points;
    source_points.push_back(Vector3f{0.0f, 0.0f, 0.0f});
    source_points.push_back(Vector3f{1.0f, 0.0f, 0.0f});
    source_points.push_back(Vector3f{0.0f, 0.0f, 2.0f});

    std::vector<Vector3f> target_points;
    target_points.push_back(Vector3f{0.1f, 0.1f, 0.1f});
    target_points.push_back(Vector3f{2.1f, -0.1f, 0.1f});

    std::unique_ptr<NearestNeighborSearch<3>> nnSearch = std::make_unique<NearestNeighborSearch<3>>();
    nnSearch->setThreshold(0.9f);
    nnSearch->buildIndex(target_points);
    std::vector<Match> matches = nnSearch->queryMatches(source_points);
    std::cout << "nMatches: " << matches.size() << std::endl;

    return 0;
}
