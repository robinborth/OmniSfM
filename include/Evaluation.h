#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include "Eigen.h"
#include "Definitions.h"
#include "Visualization.h"
#include "SfMInitializer.h"
#include "SfmGraph.h"

void computeRelativePose(const cv::Mat &R1, const cv::Mat &t1, const cv::Mat &R2, const cv::Mat &t2, cv::Mat &R_rel, cv::Mat &t_rel)
{
    if (R1.empty() || R2.empty() || t1.empty() || t2.empty())
    {
        throw std::runtime_error("One of the input matrices to computeRelativePose is empty");
    }
    double epsilon = 1e-6;
    bool isSameRotation = cv::norm(R1, R2, cv::NORM_INF) < epsilon;
    bool isSameTranslation = cv::norm(t1, t2, cv::NORM_INF) < epsilon;

    if (isSameRotation && isSameTranslation)
    {
        R_rel = cv::Mat::eye(3, 3, R1.type());   // Set to identity matrix
        t_rel = cv::Mat::zeros(3, 1, t1.type()); // Set to zero vector
    }
    else
    {
        R_rel = R1.inv() * R2;
        t_rel = R1.inv() * (t2 - t1);
    }
}

double calculateRotationError(const cv::Mat &R1, const cv::Mat &R2)
{
    if (R1.empty() || R2.empty())
    {
        throw std::runtime_error("One of the input matrices to calculateRotationError is empty");
    }
    cv::Mat diff = R1 - R2;
    return cv::norm(diff, cv::NORM_L2);
}

double calculateTranslationError(const cv::Mat &t1, const cv::Mat &t2)
{
    if (t1.empty() || t2.empty())
    {
        throw std::runtime_error("One of the input matrices to calculateTranslationError is empty");
    }
    return cv::norm((t1 - t2), cv::NORM_L2);
}

void evaluatePoseError(const Image &img1, const Image &img2, const Eigen::Matrix4f &estimatedPose, SfMInitializer &sfm)
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
    if (R_est.empty() || t_est.empty())
    {
        throw std::runtime_error("Estimated pose matrices are empty");
    }

    // Calculate the errors
    double rotation_error = calculateRotationError(R_rel, R_est);
    double translation_error = calculateTranslationError(t_rel, t_est);

    std::cout << "Rotation Error: " << rotation_error << std::endl;
    std::cout << "Translation Error: " << translation_error << std::endl;

    Visualization myVis = Visualization("myOutput2");

    cv::Mat w_r = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat w_t = cv::Mat::zeros(3, 1, CV_32F);
    Eigen::Matrix4f white = sfm.combineRotationAndTranslationIntoMatrix(w_r, w_t);

    Eigen::Matrix4f black = sfm.combineRotationAndTranslationIntoMatrix(img1.R, img1.t);
    Eigen::Matrix4f blue = sfm.combineRotationAndTranslationIntoMatrix(img2.R, img2.t);
    Eigen::Matrix4f yellow = sfm.combineRotationAndTranslationIntoMatrix(R_est, t_est);
    Eigen::Matrix4f red = sfm.combineRotationAndTranslationIntoMatrix(R_rel, t_rel);

    std::cout << "Estimated Pose: " << std::endl;
    std::cout << R_est << std::endl;
    std::cout << t_est << std::endl;
    std::cout << "Ground Truth Pose: " << std::endl;
    std::cout << R_rel << std::endl;
    std::cout << t_rel << std::endl;

    myVis.addCamera(white, 0.003, {255, 255, 255, 255});
    // myVis.addCamera(black, 0.0003, {0, 0, 0, 255});
    // myVis.addCamera(blue, 0.0003, {0, 0, 255, 255});
    myVis.addCamera(yellow, 0.003, {255, 255, 0, 255});
    // myVis.addCamera(red, 0.0003, {255, 0, 0, 255});
    myVis.writeAllMeshes();
}

void printNodeDetails(const SfMGraph& graph)
{
    std::cout << "Printing node details..." << std::endl;
    for (const auto& cam : graph.cams) {
        std::cout << "Node Pose:\n" << cam.pose << std::endl;
        std::cout << "Intrinsics:\n" << cam.intrinsics << std::endl;
        std::cout << "Keypoints count: " << cam.keypoints.size() << std::endl;
        for (const auto& kp : cam.keypoints) {
            std::cout << "Keypoint: (" << kp.pt.x << ", " << kp.pt.y << ")" << std::endl;
        }
    }
}

void printEdgeDetails(const SfMGraph& graph)
{
    std::cout << "Printing edge details..." << std::endl;
    for (const auto& edge : graph.edges) {
        std::cout << "Edge between Node " << edge.node1_index << " and Node " << edge.node2_index << std::endl;
        std::cout << "Match count: " << edge.matches.size() << std::endl;
        for (const auto& match : edge.matches) {
            std::cout << "Match: imgIdx1: " << match.imgIdx << ", queryIdx: " << match.queryIdx << ", trainIdx: " << match.trainIdx << std::endl;
        }
    }
}

void printPoint3DDetails(const SfMGraph& graph)
{
    std::cout << "Printing 3D point details..." << std::endl;
    for (const auto& point3D : graph.point3DList) {
        std::cout << "3D Point: (" << point3D.position(0) << ", " << point3D.position(1) << ", " << point3D.position(2) << ")" << std::endl;
        std::cout << "Observations count: " << point3D.observations.size() << std::endl;
        for (const auto& obs : point3D.observations) {
            std::cout << "Observation: Node index: " << obs.first << ", Keypoint index: " << obs.second << std::endl;
        }
    }
}

// std::map<ImagePair, MatchList> ImagePairMatches

void printAllMatches(ImagePairMatches allMatches)
{
    for (const auto& [pair, matches] : allMatches) 
    {
        std::cout << "Matches between images " << pair.first << " and " << pair.second << std::endl;
        std::cout << "Number of matches: " << matches.size() << std::endl;
        std::cout << "##########################################################" << std::endl;
        // for (const auto& match : matches) 
        // {
        //     std::cout << "QueryIdx: " << match.queryIdx << ", trainIdx: " << match.trainIdx << std::endl;
        // }
        // std::cout << "##########################################################" << std::endl;
    }
}
