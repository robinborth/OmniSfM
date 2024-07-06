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
#include <opencv2/core/eigen.hpp>

#include "include/BundleAdjustment.h"

void computeRelativePose(const cv::Mat &R1, const cv::Mat &t1, const cv::Mat &R2, const cv::Mat &t2, cv::Mat &R_rel, cv::Mat &t_rel)
{
    if (R1.empty() || R2.empty() || t1.empty() || t2.empty())
    {
        throw std::runtime_error("One of the input matrices to computeRelativePose is empty");
    }
    double epsilon = 1e-6;
    bool isSameRotation = cv::norm(R1, R2, cv::NORM_INF) < epsilon;
    bool isSameTranslation = cv::norm(t1, t2, cv::NORM_INF) < epsilon;

    if (isSameRotation && isSameTranslation) {
        R_rel = cv::Mat::eye(3, 3, R1.type()); // Set to identity matrix
        t_rel = cv::Mat::zeros(3, 1, t1.type()); // Set to zero vector
    } else {
        R_rel = R1.inv() * R2;
        t_rel = R1.inv() * (t2 - t1);
    }
}

double calculateRotationError(const cv::Mat& R1, const cv::Mat& R2) 
{
    if (R1.empty() || R2.empty()) {
        throw std::runtime_error("One of the input matrices to calculateRotationError is empty");
    }
    cv::Mat diff = R1 - R2;
    return cv::norm(diff, cv::NORM_L2);
}

double calculateTranslationError(const cv::Mat& t1, const cv::Mat& t2) 
{
    if (t1.empty() || t2.empty()) {
        throw std::runtime_error("One of the input matrices to calculateTranslationError is empty");
    }
    return cv::norm((t1 - t2), cv::NORM_L2);
}

void evaluatePoseError(const Image& img1, const Image& img2, const Eigen::Matrix4f& estimatedPose) 
{
    // Compute the ground truth relative pose"
    cv::Mat R_rel, t_rel;

    computeRelativePose(img1.R, img1.t, img2.R, img2.t, R_rel, t_rel);

    // Extract the estimated relative pose
    cv::Mat R_est;
    cv::Mat t_est;
    Eigen::Matrix3f r_est_e = estimatedPose.block<3, 3>(0, 0);
    Eigen::Vector3f t_est_e = estimatedPose.block<3, 1>(0, 3);
    cv::eigen2cv(r_est_e, R_est);
    cv::eigen2cv(t_est_e, t_est);


    // Check if the estimated pose matrices are populated correctly
    if (R_est.empty() || t_est.empty()) {
        throw std::runtime_error("Estimated pose matrices are empty");
    }

    // Calculate the errors
    double rotation_error = calculateRotationError(R_rel, R_est);
    double translation_error = calculateTranslationError(t_rel, t_est);

    std::cout << "Rotation Error: " << rotation_error << std::endl;
    std::cout << "Translation Error: " << translation_error << std::endl;

    Visualization myVis = Visualization("myOutput2");
    SfMInitializer sfm;

    cv::Mat w_r = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat w_t = cv::Mat::zeros(3, 1, CV_32F);
    Eigen::Matrix4f white = sfm.combineRotationAndTranslationIntoMatrix(w_r, w_t);


    Eigen::Matrix4f black = sfm.combineRotationAndTranslationIntoMatrix(img1.R, img1.t);
    Eigen::Matrix4f blue = sfm.combineRotationAndTranslationIntoMatrix(img2.R, img2.t);
    Eigen::Matrix4f yellow = sfm.combineRotationAndTranslationIntoMatrix(R_est, t_est);
    Eigen::Matrix4f red = sfm.combineRotationAndTranslationIntoMatrix(R_rel, t_rel);

    std::cout << "Estimated Pose: " << std::endl;
    std::cout << R_est  << std::endl;
    std::cout << t_est  << std::endl;
    std::cout << "Ground Truth Pose: " << std::endl;
    std::cout << R_rel  << std::endl;
    std::cout << t_rel  << std::endl;
    
    myVis.addCamera(white, 0.0003, {255, 255, 255, 255});
    myVis.addCamera(black, 0.0003, {0, 0, 0, 255});
    myVis.addCamera(blue, 0.0003, {0, 0, 255, 255});
    myVis.addCamera(yellow, 0.0003, {255, 255, 0, 255});
    myVis.addCamera(red, 0.0003, {255, 0, 0, 255});
    myVis.writeAllMeshes();
}

void visualizeCorrespondencesBwTwoImg(int id1, int id2, CorrespondenceSearch &search, ImageStorage &imageStorage)
{
    std::cout << "==> Visualize correspondences between id" << id1 << " and id" << id2 << std::endl;

    Image* sImg = imageStorage.findImage(id1);
    Image* tImg = imageStorage.findImage(id2);
    std::vector<cv::DMatch> matches = search.queryMatches(*sImg, *tImg);
    cv::Mat correspondenceImage = search.visualizeCorrespondences(*sImg, *tImg, matches);
    std::string filename = "correspondenceImage" + std::to_string(id1) + "_" + std::to_string(id2) + ".jpg";

    cv::imwrite(filename, correspondenceImage);
    std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;

    std::cout << "==> Visualize inliers between id=0 and id=1 ..." << std::endl;
    auto inlierMatches = search.filterMatchesWithRANSAC(*sImg, *tImg, matches);
    cv::Mat inlierImage = search.visualizeCorrespondences(*sImg, *tImg, inlierMatches);
    filename = "inlierImage" + std::to_string(id1) + "_" + std::to_string(id2) + ".jpg";
    cv::imwrite(filename, inlierImage);
    std::cout << "==> Found " << inlierMatches.size() << " inlier matches ..." << std::endl;
}

void initializePoseBwTwoImages(int id1, int id2, CorrespondenceSearch &search, ImageStorage &imageStorage)
{
    auto matches = search.queryMatches(imageStorage.images[id1], imageStorage.images[id2]);
    std::cout << "==> Found " << matches.size() << " matches ..." << std::endl;
    auto inlierMatches = search.filterMatchesWithRANSAC(imageStorage.images[id1], imageStorage.images[id2], matches);
    std::cout << "==> Found " << inlierMatches.size() << " inlier matches ..." << std::endl;
    SfMInitializer sfm;
    Eigen::Matrix4f cameraPose = sfm.debugRunSfm(imageStorage.images, matches, id1, id2);
    
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
    CorrespondenceSearch search;
    SfMInitializer sfm;
    int id1 = 0;
    int id2 = 1;
    //visualizeCorrespondencesBwTwoImg(id1, id2, search, imageStorage);
    //initializePoseBwTwoImages(id1, id2, search, imageStorage);
    std::cout << "==> Find correspondences ..." << std::endl;
    auto allMatches = search.queryCorrespondences(imageStorage.images);

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

    Image* img1 = imageStorage.findImage(id1);
    Image* img2 = imageStorage.findImage(id2);

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

    return 0;
}
