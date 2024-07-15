#include <iostream>

#include "include/Settings.h"
#include "include/Eigen.h"
#include "include/ImageStorage.h"
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
    imageStorage.useEveryNthImage(4); // current setting uses --n=51
    std::cout << imageStorage.getNumImages() << " images will be used." << std::endl;

    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();

    std::cout << "==> Find correspondences ..." << std::endl;
    Image targetImg;
    Image sourceImg;
    SfMInitializer sfm(imageStorage, sfmGraph);
    CorrespondenceSearch search;

    sourceImg = imageStorage.images[0];                   // initilize with the first image
    for (auto k = 1; k < imageStorage.images.size(); ++k) // iterate over all the images
    {
        // extract the next target image
        if (k > 1) // skip the first iteration for the update
            sourceImg = targetImg;
        targetImg = imageStorage.images[k];

        // searches for inliers between the current current image and the source
        auto matches = search.queryMatches(sourceImg, targetImg);
        auto inlierMatches = search.filterMatchesWithRANSAC(sourceImg, targetImg, matches);

        if (k == 1)
        {
            std::cout << "==> Essential Matrix Initialization ..." << std::endl;
            sfm.twoViewSfm(inlierMatches, sourceImg.id, targetImg.id);
        }
        else
        {
            std::cout << "==> PnP Initilization..." << std::endl;
            sfm.addSfM(inlierMatches, sourceImg.id, targetImg.id);
        }

        // debug the projections before
        debugProjection(targetImg, sfmGraph, "before");
        std::cout << "Reprojection Error " << calculateReprojectionError(sourceImg, targetImg, sfmGraph) << std::endl;
        // run bundle adjustment
        std::cout << "==> Local Bundle Adjustment step (" << k << ") ..." << std::endl;
        bundleAdjustment.Adjust(sfmGraph);
        // debug the projections after
        debugProjection(targetImg, sfmGraph, "after");
        std::cout << "Reprojection Error " << calculateReprojectionError(sourceImg, targetImg, sfmGraph) << std::endl;
    }

    std::cout << "==> Visualize SfM ..." << std::endl;
    Visualization sfmVis = Visualization("sfm");
    std::vector<Vertex> points3D = sfm.getPoints3D();
    // this is cheating but for visualization better
    // we should remove outliers in SfM!
    std::vector<Vertex> cleanPoints3D = cleanPointCloud(points3D);

    auto cameraPoses = sfm.getCameraPoses();
    sfmVis.addVertex(cleanPoints3D);
    sfmVis.addCamera(cameraPoses, 0.0002);
    sfmVis.writeAllMeshes();
    for (size_t i = 0; i < cameraPoses.size(); ++i)
    {
        std::cout << "Camera Pose Before BA " << i + 1 << ":\n";
        std::cout << cameraPoses[i] << "\n\n";
    }

    std::cout << "Reprojection Error " << calculateReprojectionError(sourceImg, targetImg, sfmGraph) << std::endl;

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
