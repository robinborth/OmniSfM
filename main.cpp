#include <iostream>

#include "src/Settings.h"
#include "src/Eigen.h"
#include "src/ImageStorage.h"
#include "src/SfMOptimizer.h"
#include "src/PointCloud.h"
#include "src/CorrespondenceSearch.h"
#include "src/Utils.h"
#include "src/SfMInitializer.h"
#include "src/SimpleMesh.h"
#include "src/Visualization.h"



int main()
{
    std::cout << "==> Load settings ..." << std::endl;
    Settings settings;

    std::cout << "==> Creating image store ..." << std::endl;
    ImageStorage imageStorage(settings.dataDir);

    std::cout << "Initialize virtual sensor..." << std::endl;
    VirtualSensor sensor;
    if (!sensor.Init(settings.dataDir))
    {
        std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
        return -1;
    }

    cv::Mat K = getIntrinsicsFromSensor(sensor);

    std::cout << "==> Load images ..." << std::endl;
    imageStorage.loadImages();

    std::cout << "==> Detect keypoints ..." << std::endl;
    imageStorage.detectKeypoints();
    // imageStorage.drawKeypoints(1305031128747363, "keypoints00.jpg");
    // imageStorage.drawKeypoints(1305031128711457, "keypoints01.jpg");

    std::cout << "==> Find correspondences ..." << std::endl;
    CorrespondenceSearch search;
    // Do not delete this code snippet
    // auto &sImg = imageStorage.images[796];
    // auto &tImg = imageStorage.images[797];

    // std::vector<Match> matches = search.queryMatches(sImg, tImg);
    // std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;

    // auto inlierMatches = search.filterMatchesWithRANSAC(sImg, tImg, matches);
    // std::cout << "==> Found " << inlierMatches.size() << " inlier matches ..." << std::endl;

    // // Only keep pairs with at least 60 matches
    // if (inlierMatches.size() >= 60)
    // {
    //     std::cout << "==> Visualize correspondences ..." << std::endl;
    //     cv::Mat correspondenceImage = search.visualizeCorrespondences(sImg, tImg, inlierMatches);
    //     cv::imwrite("correspondenceImage.jpg", correspondenceImage);
    // }

    // While loading images you can adjust the number of images to load
    // For the whole dataset there are 789 images and it will take too long
    // Maybe choose something like 100-200 images
    auto allMatches = search.queryCorrespondences(imageStorage.images);

    // Create an instance of StructureFromMotion
    SfMInitializer sfm(K);

    // Run Structure from Motion
    sfm.runSfM(imageStorage.images, allMatches);
    const auto& points3D = sfm.getPoints3D();

    // Save points to file for visualization
    std::string outputFilename = "point_cloud.xyz";
    savePointsToFile(points3D, outputFilename);

    std::cout << "Point cloud saved to " << outputFilename << std::endl;

    Visualization myVis = Visualization("myOutput");

    myVis.addVertex(points3D);
    //myVis.addCamera()
    myVis.fillCamerasWithDefaultValues();
    myVis.fillPointCloudWithDefaultValues();
    myVis.writeAllMeshes();


    // std::cout << "(TODO) ==> Optimize SfM ..." << std::endl;

    // std::cout << "(TODO) ==> Create dense point cloud ..." << std::endl;

    // std::cout << "(TODO) ==> Create mesh ..." << std::endl;

    // std::cout << "(TODO) ==> Save mesh ..." << std::endl;

    return 0;
}
