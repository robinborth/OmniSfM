#include <iostream>

#include "src/settings.h"
#include "src/Eigen.h"
#include "src/ImageStorage.h"
#include "src/SfMOptimizer.h"
#include "src/PointCloud.h"
#include "src/NearestNeighbor.h"
#include "src/Utils.h"

int main()
{
    std::cout << "==> Load settings ..." << std::endl;
    Settings settings;

    std::cout << "==> Creating image store ..." << std::endl;
    ImageStorage imageStorage(settings.dataDir);

    std::cout << "==> Load images ..." << std::endl;
    imageStorage.loadImages();

    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();
    imageStorage.drawKeypoints(0, "keypoints00.jpg");
    imageStorage.drawKeypoints(1, "keypoints01.jpg");

    std::cout << "==> Build feature index ..." << std::endl;
    NearestNeighborSearch<128> nnSearch = NearestNeighborSearch<128>();
    nnSearch.setThreshold(settings.siftThreshold);
    nnSearch.buildIndex(imageStorage.images[1]);
    std::cout << "==> Find correspondences ..." << std::endl;
    std::vector<Match> matches = nnSearch.queryMatches(imageStorage.images[0]);
    std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;

    std::cout << "==> Visualize correspondences ..." << std::endl;
    cv::Mat correspondenceImage = visualizeCorrespondences(imageStorage.images[0], imageStorage.images[1], matches);
    cv::imwrite("correspondenceImage.jpg", correspondenceImage);

    std::cout << "(TODO) ==> Optimize SfM ..." << std::endl;

    std::cout << "(TODO) ==> Create dense point cloud ..." << std::endl;

    std::cout << "(TODO) ==> Create mesh ..." << std::endl;

    std::cout << "(TODO) ==> Save mesh ..." << std::endl;

    return 0;
}
