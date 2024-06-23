#include <iostream>

#include "include/Settings.h"
#include "include/Eigen.h"
#include "include/ImageStorage.h"
#include "include/SfMOptimizer.h"
#include "include/PointCloud.h"
#include "include/CorrespondenceSearch.h"
#include "include/Utils.h"
#include "include/SfMInitializer.h"
#include "include/SimpleMesh.h"
#include "include/Visualization.h"
#include "include/Definitions.h"

#include <opencv2/viz.hpp>

void visualizePointCloud(const std::vector<cv::Point3f>& points) {
    cv::viz::Viz3d window("Point Cloud");

    // Convert points to cv::Mat
    cv::Mat pointCloudMat = cv::Mat(points).reshape(3, points.size());

    // Create a cloud widget
    cv::viz::WCloud cloudWidget(pointCloudMat, cv::viz::Color::green());

    window.showWidget("Cloud", cloudWidget);
    window.spin();
}


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
    // imageStorage.drawKeypoints(0, "keypoints00000.jpg");
    // imageStorage.drawKeypoints(1, "keypoints00001.jpg");

    // TODO Do here the visualization of the GT point cloud
    // 1) Use extrinsics and intrinsics from freiburg with depth map

    // Correspondecne search with debugging
    CorrespondenceSearch search;

    // std::cout << "==> Visualize correspondences between id=0 and id=1 ..." << std::endl;
    // auto &sImg = imageStorage.images[0];
    // auto &tImg = imageStorage.images[1];
    // std::vector<Match> matches = search.queryMatches(sImg, tImg);
    // cv::Mat correspondenceImage = search.visualizeCorrespondences(sImg, tImg, matches);
    // cv::imwrite("correspondenceImage.jpg", correspondenceImage);
    // std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;

    // std::cout << "==> Visualize inliers between id=0 and id=1 ..." << std::endl;
    // auto inlierMatches = search.filterMatchesWithRANSAC(sImg, tImg, matches);
    // cv::Mat inlierImage = search.visualizeCorrespondences(sImg, tImg, inlierMatches);
    // cv::imwrite("inlierImage.jpg", inlierImage);
    // std::cout << "==> Found " << inlierMatches.size() << " inlier matches ..." << std::endl;

    std::cout << "==> Find correspondences ..." << std::endl;
    auto allMatches = search.queryCorrespondences(imageStorage.images);

    size_t totalMatches = 0;

    // Iterate over each vector of matches and add up their sizes
    for (const auto& matches : allMatches) {
        totalMatches += matches.size();
    }

    std::cout << "Total number of matches: " << totalMatches << std::endl;

    // // Create an instance of StructureFromMotion
    SfMInitializer sfm;                // TODO this needs to handle multiple Ks for each different image

    // Run Structure from Motion
    sfm.runSfM(imageStorage.images, allMatches);
    const std::vector<ColoredPoint3f> &points3D = sfm.getPoints3D();
    const auto &cameraPoses = sfm.getCameraPoses();

    std::vector<cv::Point3f> points;
    std::vector<cv::Vec3b> colors;

    for (const auto& coloredPoint : points3D) {
        points.push_back(coloredPoint.point);
        colors.push_back(coloredPoint.color);
    }
        

    //visualizePointCloud(points3D);
    Visualization myVis = Visualization("myOutput");
    myVis.addVertex(points , colors);
    myVis.addCamera(cameraPoses);
    myVis.writeAllMeshes();

    // std::cout << "Point cloud saved to " << outputFilename << std::endl;

    // std::cout << "(TODO) ==> Optimize SfM ..." << std::endl;

    // std::cout << "(TODO) ==> Create dense point cloud ..." << std::endl;

    // std::cout << "(TODO) ==> Create mesh ..." << std::endl;

    // std::cout << "(TODO) ==> Save mesh ..." << std::endl;

    return 0;
}
