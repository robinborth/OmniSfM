#pragma once
#include "Definitions.h"
#include "SfmGraph.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <map>


void drawKeypoints_(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypoints, const std::string& windowName) 
{
    cv::Mat output;
    cv::drawKeypoints(image, keypoints, output, cv::Scalar::all(-1));
    cv::Point2f pont6 = keypoints[6].pt;
    cv::circle(image, (cv::Point) pont6, 6, cv::Scalar(0, 255, 0), -1);
    cv::imwrite(windowName + ".png", output);
}

void projectAndDraw3DPoints(
    const cv::Mat& image,
    const std::vector<Vertex>& vertices,
    const Eigen::Matrix3f& intrinsics,
    const Eigen::Matrix4f& pose,
    const std::string& windowName)
{
    std::vector<cv::Point3f> points3D;
    for (const Vertex& vertex : vertices) {
        // Convert Eigen::Vector4f to cv::Point3f, ignoring the homogeneous coordinate
        points3D.emplace_back(vertex.position[0], vertex.position[1], vertex.position[2]);
    }

    // Convert Eigen matrices to OpenCV format for camera matrix and pose transformation
    cv::Mat cameraMatrix;
    cv::eigen2cv(intrinsics, cameraMatrix);
    Eigen::Matrix3f rotationMatrix = pose.block<3, 3>(0, 0);
    Eigen::Vector3f translationVector = pose.block<3, 1>(0, 3);
    cv::Mat rvec, tvec;
    cv::Mat cvRotationMatrix;
    cv::eigen2cv(rotationMatrix, cvRotationMatrix);
    cv::Rodrigues(cvRotationMatrix, rvec);
    cv::eigen2cv(translationVector, tvec);

    // Project the 3D points to the 2D image
    std::vector<cv::Point2f> projectedPoints;
    // opencv function to project 3D points to 2D
    cv::projectPoints(points3D, rvec, tvec, cameraMatrix, cv::Mat(), projectedPoints);

    // Draw projected points on the image
    cv::Mat output = image.clone();
    for (const auto& pt : projectedPoints) {
        cv::circle(output, pt, 4, cv::Scalar(0, 255, 0), -1); // Green points for visibility
    }

    // Display the result
    cv::imwrite(windowName + ".png", output);
}

// Function to project and draw 3D points onto 2D image and link them to 2D keypoints
void projectAndDraw3DPointsAndKeypoints(
    const cv::Mat& image,
    const std::vector<Point3D>& vertices,
    const std::vector<cv::KeyPoint>& keypoints,
    const std::vector<cv::DMatch>& matches, // Matches linking 3D points to 2D keypoints
    const Eigen::Matrix3f& intrinsics,
    const Eigen::Matrix4f& pose,
    const std::string& windowName)
{
    // Project the 3D points to the 2D image manually
    std::vector<cv::Point2f> projectedPoints;
    for (int i =0; i < vertices.size(); ++i)
    {
        Eigen::Vector4f position = vertices[i].position;
        Eigen::Vector4f transformedPoint = pose * position;
        Eigen::Vector3f point = transformedPoint.head<3>();
        // fx * xp + cx
        Eigen::Vector3f projectedPoint = intrinsics * point;
        projectedPoint /= projectedPoint[2];
        Eigen::Vector2f projectedPoint2D = projectedPoint.head<2>();
        projectedPoints.push_back(cv::Point2f(projectedPoint2D[0], projectedPoint2D[1]));
    }

    cv::Mat output = image.clone();
    std::cout << "==> Drawing " << projectedPoints.size() << " 3D points and " << keypoints.size() << " 2D keypoints" << std::endl;
    int i=0;
    for (const auto &m : matches)
    {
        cv::circle(output, keypoints[m.queryIdx].pt, 4, cv::Scalar(0, 255, 0), -1); // Green points for 2D
        cv::circle(output, projectedPoints[i], 4, cv::Scalar(0, 0, 255), -1); // Blue points for 3D
        i++;
    }
    // Display the result
    cv::imwrite(windowName + ".png", output);
}

void debugProjectionfor2viewSfm(const cv::Mat& image1,
                    const cv::Mat& image2,
                    SfMGraph& graph)
{
    const auto& cam1 = graph.cams[0];
    const auto& cam2 = graph.cams[1];
    const auto& edge1 = graph.edges[0];

    projectAndDraw3DPointsAndKeypoints(image1, graph.point3DList, cam1.keypoints, edge1.matches, cam1.intrinsics, cam1.pose, "Camera_1_Projection");
    drawKeypoints_(image1, cam1.keypoints, "Camera_1_Keypoints");
    drawKeypoints_(image2, cam2.keypoints, "Camera_2_Keypoints");

}
