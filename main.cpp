#include <iostream>

#include "src/Settings.h"
#include "src/Eigen.h"
#include "src/ImageStorage.h"
#include "src/SfMOptimizer.h"
#include "src/PointCloud.h"
#include "src/CorrespondenceSearch.h"
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

    std::cout << "==> Find correspondences ..." << std::endl;
    auto &sImg = imageStorage.images[0];
    auto &tImg = imageStorage.images[1];
    CorrespondenceSearch<128> search = CorrespondenceSearch<128>();
    search.setThreshold(settings.siftThreshold);
    std::vector<Match> matches = search.queryMatches(sImg, tImg);
    std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;

    std::cout << "==> Visualize correspondences ..." << std::endl;
    cv::Mat correspondenceImage = visualizeCorrespondences(sImg, tImg, matches);
    cv::imwrite("correspondenceImage.jpg", correspondenceImage);

    std::cout << "(TODO) ==> Optimize SfM ..." << std::endl;

    std::cout << "(TODO) ==> Create dense point cloud ..." << std::endl;

    std::cout << "(TODO) ==> Create mesh ..." << std::endl;

    std::cout << "(TODO) ==> Save mesh ..." << std::endl;

    return 0;
}
