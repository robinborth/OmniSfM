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
#include "include/SfmGraph.h"
#include "include/BundleAdjustment.h"
#include "include/Evaluation.h"
#include <Eigen/Dense>
#include "include/Debug.h"
#include <opencv2/core/eigen.hpp>


int main()
{
    std::cout << "==> Load settings ..." << std::endl;
    Settings settings;
    std::cout << "==> Creating image store ..." << std::endl;
    ImageStorage imageStorage(settings);
    SfMGraph sfmGraph;
    BundleAdjustment bundleAdjustment;
    std::cout << "==> Load images ..." << std::endl;
    imageStorage.loadImages();

    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();

    std::cout << "==> Find correspondences ..." << std::endl;
    CorrespondenceSearch search;
    Image *img0 = imageStorage.findImage(0);
    Image *img1 = imageStorage.findImage(6);
    //search.visualizeCorrespondencesBwTwoImg(*img0, *img1);
    //auto matches = search.queryCorrespondences(imageStorage.images);
    auto match = search.queryMatches(*img0, *img1);
    auto inlierMatches = search.filterMatchesWithRANSAC(*img0, *img1, match);

    std::cout << "==> Run SfM ..." << std::endl;
    SfMInitializer sfm(imageStorage, sfmGraph);
    //sfm.runSfM(matches);
    sfm.twoViewSfm(inlierMatches, 0, 6);

    bundleAdjustment.Adjust(sfmGraph);

    // std::cout << "==> Visualize SfM ..." << std::endl;
    // Visualization sfmVis = Visualization("sfm");
    // std::vector<Vertex> points3D = sfm.getPoints3D();
    // auto cameraPoses = sfm.getCameraPoses();
    // sfmVis.addVertex(points3D);
    // sfmVis.addCamera(cameraPoses, 0.001);
    // sfmVis.writeAllMeshes();

    // std::cout << "==> Visualize MVS ..." << std::endl;
    // Visualization mvsVis = Visualization("mvs");
    // cameraPoses = sfm.getCameraPoses();
    // mvsVis.addCamera(cameraPoses, 0.001);
    // for (auto &img : imageStorage.images)
    // {
    //     std::cout << "Add image (" << img.id << ") to the visualiztion." << std::endl;
    //     std::vector<Vertex> verticies = extractPointCloud(img);
    //     mvsVis.addVertex(verticies);
    // }
    // std::vector<Vertex> verticies = extractPointCloud(*img0);
    // mvsVis.addVertex(verticies);
    // verticies = extractPointCloud(*img1);
    // mvsVis.addVertex(verticies);

    // mvsVis.writeAllMeshes();

    return 0;
}
