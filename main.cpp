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
#include <Eigen/Dense>


void computeRelativePose(const cv::Mat& R1, const cv::Mat& t1, const cv::Mat& R2, const cv::Mat& t2, cv::Mat& R_rel, cv::Mat& t_rel) 
{
    if (R1.empty() || R2.empty() || t1.empty() || t2.empty()) {
        throw std::runtime_error("One of the input matrices to computeRelativePose is empty");
    }
    R_rel = R2 * R1.t();
    t_rel = t2 - R_rel * t1;
}

double calculateRotationError(const cv::Mat& R1, const cv::Mat& R2) 
{
    if (R1.empty() || R2.empty()) {
        throw std::runtime_error("One of the input matrices to calculateRotationError is empty");
    }
    cv::Mat diff = R1 - R2;
    return cv::norm(diff);
}

double calculateTranslationError(const cv::Mat& t1, const cv::Mat& t2) 
{
    if (t1.empty() || t2.empty()) {
        throw std::runtime_error("One of the input matrices to calculateTranslationError is empty");
    }
    return cv::norm(t1 - t2);
}

void evaluatePoseError(const Image& img1, const Image& img2, const Eigen::Matrix4f& estimatedPose) 
{
    // Compute the ground truth relative pose
    cv::Mat R_rel, t_rel;
    computeRelativePose(img1.R, img1.t, img2.R, img2.t, R_rel, t_rel);

    // Extract the estimated relative pose
    cv::Mat R_est_rel(3, 3, CV_32F);
    cv::Mat t_est_rel(3, 1, CV_32F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_est_rel.at<float>(i, j) = estimatedPose(i, j);
        }
        t_est_rel.at<float>(i, 0) = estimatedPose(i, 3);
    }

    // Check if the estimated pose matrices are populated correctly
    if (R_est_rel.empty() || t_est_rel.empty()) {
        throw std::runtime_error("Estimated pose matrices are empty");
    }

    // Calculate the errors
    double rotation_error = calculateRotationError(R_rel, R_est_rel);
    double translation_error = calculateTranslationError(t_rel, t_est_rel);

    std::cout << "Rotation Error: " << rotation_error << std::endl;
    std::cout << "Translation Error: " << translation_error << std::endl;
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

    Image* img1 = imageStorage.findImage(0);
    Image* img2 = imageStorage.findImage(1);

    if (!img1 || !img2) {
        std::cerr << "Failed to find one of the images." << std::endl;
        return -1;
    }
    if (cameraPoses.empty()) {
        std::cerr << "No estimated poses found." << std::endl;
        return -1;
    }
    const Eigen::Matrix4f& estimatedPose = cameraPoses[0];

    // Evaluate the pose error
    evaluatePoseError(*img1, *img2, estimatedPose);

        

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
