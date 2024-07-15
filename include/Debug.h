#pragma once
#include "Definitions.h"
#include "SfmGraph.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <map>

void drawKeypoints_(const cv::Mat &image, const std::vector<cv::KeyPoint> &keypoints, const std::string &windowName)
{
    cv::Mat output;
    cv::drawKeypoints(image, keypoints, output, cv::Scalar::all(-1));
    cv::imwrite(windowName + ".png", output);
}

void projectAndDraw3DPoints(
    const cv::Mat &image,
    const std::vector<Vertex> &vertices,
    const Eigen::Matrix3f &intrinsics,
    const Eigen::Matrix4f &pose,
    const std::string &windowName)
{
    std::vector<cv::Point3f> points3D;
    for (const Vertex &vertex : vertices)
    {
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
    for (const auto &pt : projectedPoints)
    {
        cv::circle(output, pt, 4, cv::Scalar(0, 255, 0), -1); // Green points for visibility
    }

    // Display the result
    cv::imwrite(windowName + ".png", output);
}

Eigen::Vector2f projectPoint3Dto2D(
    const cv::Mat image,
    const Eigen::Vector4f position,
    const Eigen::Matrix3f intrinsics,
    const Eigen::Matrix4f pose)
{
    Eigen::Vector4f transformedPoint = pose * position;
    Eigen::Vector3f point = transformedPoint.head<3>();
    // fx * xp + cx
    Eigen::Vector3f projectedPoint = intrinsics * point;
    projectedPoint /= projectedPoint[2];
    return projectedPoint.head<2>();
}

// Function to project and draw 3D points onto 2D image and link them to 2D keypoints
void projectAndDraw3DPointsAndKeypoints(
    const Image &image,
    const std::vector<Point3D> &vertices,
    const std::vector<cv::KeyPoint> &keypoints,
    const std::vector<cv::DMatch> &matches, // Matches linking 3D points to 2D keypoints
    const Eigen::Matrix3f &intrinsics,
    const Eigen::Matrix4f &pose,
    const std::string &windowName)
{
    cv::Mat output = image.rgb.clone();
    // Project the 3D points to the 2D image manually
    for (const auto &v : vertices)
    {
        Eigen::Vector4f position = v.position;
        Eigen::Vector2f projectedPoint2D = projectPoint3Dto2D(image.rgb, position, intrinsics, pose);
        for (const auto &obs : v.observations)
        {
            if (obs.first == image.id)
            {
                cv::circle(output, keypoints[obs.second].pt, 4, cv::Scalar(0, 255, 0), -1); // Green points for 2D
            }
        }
        cv::circle(output, cv::Point2f(projectedPoint2D[0], projectedPoint2D[1]), 4, cv::Scalar(0, 0, 255), -1); // Blue points for 3D
    }
    // Display the result
    cv::imwrite(windowName + ".png", output);
}

void debugProjectionfor2viewSfm(const Image &image1,
                                const Image &image2,
                                SfMGraph &graph)
{
    const auto &cam1 = graph.cams[0];
    const auto &cam2 = graph.cams[1];
    const auto &edge1 = graph.edges[0];

    projectAndDraw3DPointsAndKeypoints(image1, graph.point3DList, cam1.keypoints, edge1.matches, cam1.intrinsics, cam1.pose, "Camera_1_Projection");
    projectAndDraw3DPointsAndKeypoints(image2, graph.point3DList, cam2.keypoints, edge1.matches, cam2.intrinsics, cam2.pose, "Camera_2_Projection");
    drawKeypoints_(image1.rgb, cam1.keypoints, "Camera_1_Keypoints");
    drawKeypoints_(image2.rgb, cam2.keypoints, "Camera_2_Keypoints");
}

void debugProjection(const Image &img, SfMGraph &graph, std::string prefix)
{
    Node camera;
    bool existCamera = false;
    for (auto &cam : graph.cams)
    {
        if (cam.id == img.id)
        {
            camera = cam;
            existCamera = true;
        }
    }
    if (!existCamera)
        std::cout << "ERROR: Camera does not exists in debugProjectionfor2viewSfm" << std::endl;

    std::vector<cv::DMatch> matches; // we don't need that
    std::string name = "camera_" + std::to_string(img.id) + "_projection_" + prefix;
    projectAndDraw3DPointsAndKeypoints(img, graph.point3DList, camera.keypoints, matches, camera.intrinsics, camera.pose, name);
    name = "Camera_" + std::to_string(img.id) + "_Keypoints_" + prefix;
    drawKeypoints_(img.rgb, camera.keypoints, name);
}

float calculateReprojectionError(const Image &image1,
                                 const Image &image2,
                                 SfMGraph &graph)
{
    const auto &cam1 = graph.cams[0];
    const auto &cam2 = graph.cams[1];
    const auto &edge1 = graph.edges[0];

    float totalError = 0.0f;
    int count = 0;
    for (const auto &point3d : graph.point3DList)
    {
        for (const auto &obs : point3d.observations)
        {
            if (obs.first == image1.id)
            {
                Eigen::Vector2f projectedPoint = projectPoint3Dto2D(image1.rgb, point3d.position, cam1.intrinsics, cam1.pose);
                cv::Point2f projectedPointCv(projectedPoint[0], projectedPoint[1]);
                totalError += cv::norm(projectedPointCv - cam1.keypoints[obs.second].pt);
                count++;
            }
            else if (obs.first == image2.id)
            {
                Eigen::Vector2f projectedPoint = projectPoint3Dto2D(image2.rgb, point3d.position, cam2.intrinsics, cam2.pose);
                cv::Point2f projectedPointCv(projectedPoint[0], projectedPoint[1]);
                totalError += cv::norm(projectedPointCv - cam2.keypoints[obs.second].pt);
                count++;
            }
        }
    }
    return totalError / count;
}

std::vector<Vertex> cleanPointCloud(std::vector<Vertex> points3D)
{

    std::vector<Vertex> _points3D;
    Eigen::VectorXf p(points3D.size());
    for (size_t i = 0; i < points3D.size(); ++i)
    {
        p(i) = points3D[i].position[2];
    }
    float mean = p.mean();
    float variance = (p.array() - mean).square().sum() / (p.size() - 1);
    float stddev = std::sqrt(variance);

    for (auto i = 0; i < points3D.size(); ++i)
    {
        auto depth = points3D[i].position[2];
        float gt_std = std::abs(depth - mean) / stddev;
        if (gt_std > 1.0 or depth < 0.0) // if the z-value is further away then 1 std this could be an outlier or negative
        {
            continue;
        }
        _points3D.push_back(points3D[i]);
    }
    return _points3D;
}