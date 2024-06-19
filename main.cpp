#include <iostream>

#include "src/Settings.h"
#include "src/Eigen.h"
#include "src/ImageStorage.h"
#include "src/SfMOptimizer.h"
#include "src/PointCloud.h"
#include "src/CorrespondenceSearch.h"
#include "src/Utils.h"
#include "src/SfMInitializer.h"

int main()
{
    // Loading the settings and the Images
    std::cout << "==> Load settings ..." << std::endl;
    Settings settings;
    std::cout << "==> Creating image store ..." << std::endl;
    ImageStorage imageStorage(settings);
    std::cout << "==> Load images ..." << std::endl;
    imageStorage.loadImages();

    // SIFT feature detection
    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();
    imageStorage.drawKeypoints(0, "keypoints00000.jpg");
    imageStorage.drawKeypoints(1, "keypoints00001.jpg");

    // TODO Do here the visualization of the GT point cloud
    // 1) Use extrinsics and intrinsics from freiburg with depth map

    // Correspondecne search with debugging
    CorrespondenceSearch search;

    std::cout << "==> Visualize correspondences between id=0 and id=1 ..." << std::endl;
    auto &sImg = imageStorage.images[0];
    auto &tImg = imageStorage.images[1];
    std::vector<Match> matches = search.queryMatches(sImg, tImg);
    cv::Mat correspondenceImage = search.visualizeCorrespondences(sImg, tImg, matches);
    cv::imwrite("correspondenceImage.jpg", correspondenceImage);
    std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;

    std::cout << "==> Visualize inliers between id=0 and id=1 ..." << std::endl;
    auto inlierMatches = search.filterMatchesWithRANSAC(sImg, tImg, matches);
    cv::Mat inlierImage = search.visualizeCorrespondences(sImg, tImg, inlierMatches);
    cv::imwrite("inlierImage.jpg", inlierImage);
    std::cout << "==> Found " << inlierMatches.size() << " inlier matches ..." << std::endl;

    std::cout << "==> Find correspondences ..." << std::endl;
    auto allMatches = search.queryCorrespondences(imageStorage.images);
    std::cout << "==> Found " << allMatches.size() << " matches ..." << std::endl;

    // // Create an instance of StructureFromMotion
    // cv::Mat K = imageStorage.images[0].K; // assume that all the Ks are the same
    // SfMInitializer sfm(K);                // TODO this needs to handle multiple Ks for each different image

    // // Run Structure from Motion
    // sfm.runSfM(imageStorage.images, allMatches);
    // const auto &points3D = sfm.getPoints3D();

    // // Save points to file for visualization
    // std::string outputFilename = "point_cloud.xyz";
    // savePointsToFile(points3D, outputFilename);

    // std::cout << "Point cloud saved to " << outputFilename << std::endl;

    // std::cout << "(TODO) ==> Optimize SfM ..." << std::endl;

    // std::cout << "(TODO) ==> Create dense point cloud ..." << std::endl;

    // std::cout << "(TODO) ==> Create mesh ..." << std::endl;

    // std::cout << "(TODO) ==> Save mesh ..." << std::endl;

    return 0;
}
