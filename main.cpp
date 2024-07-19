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
    imageStorage.useEveryNthImage(settings.useEveryNthImage);
    std::cout << imageStorage.getNumImages() << " images will be used." << std::endl;

    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();

    std::cout << "==> Find correspondences ..." << std::endl;
    Image targetImg;
    Image sourceImg;
    SfMInitializer sfm(imageStorage, sfmGraph);
    CorrespondenceSearch search;

    sourceImg = imageStorage.images[0];                   // initilize with the first image

    auto allMatches = search.queryCorrespondences(imageStorage.images);

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
        if (inlierMatches.size() < 60)
        {
            std::cout << "Not enough inliers found. Skipping this pair." << std::endl;
            continue;
        }

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
        std::cout << "Reprojection Error " << calculateOverallReprojectionError(sfmGraph, imageStorage) << std::endl;
        // run bundle adjustment
        std::cout << "==> Local Bundle Adjustment step (" << k << ") ..." << std::endl;
        bundleAdjustment.Adjust(sfmGraph);
        // debug the projections after
        debugProjection(targetImg, sfmGraph, "after");
        std::cout << "Reprojection Error " << calculateOverallReprojectionError(sfmGraph, imageStorage) << std::endl;
    }

    std::cout << "==> Global Bundle Adjustment ..." << std::endl;
    bundleAdjustment.Adjust(sfmGraph);

    return 0;
}
