#pragma once

#include <opencv2/opencv.hpp>
#include "ImageStorage.h"
#include "CorrespondenceSearch.h"
#include "VirtualSensor.h"

cv::Mat visualizeCorrespondences(Image source, Image target, std::vector<Match> matches)
{
    // Create a new image with combined width and the same height
    cv::Mat combinedImage(source.rgb.rows, source.rgb.cols + target.rgb.cols, CV_8UC3);
    // Copy the first image to the left part of the new image
    source.rgb.copyTo(combinedImage(cv::Rect(0, 0, source.rgb.cols, source.rgb.rows)));
    // Copy the second image to the right part of the new image
    target.rgb.copyTo(combinedImage(cv::Rect(source.rgb.cols, 0, target.rgb.cols, source.rgb.rows)));

    for (auto &match : matches)
    {
        // source pixel coordinates
        int xs = source.keypoints[match.sourceKeypointId].pt.x;
        int ys = source.keypoints[match.sourceKeypointId].pt.y;
        // target pixel coordinates
        int xt = target.keypoints[match.targetKeyopintId].pt.x + source.rgb.cols;
        int yt = target.keypoints[match.targetKeyopintId].pt.y;

        // Define the start and end points of the line
        cv::Point startPoint(xs, ys);
        cv::Point endPoint(xt, yt);
        // Define the color of the line (B, G, R)
        cv::Scalar lineColor(0, 0, 255); // Red color
        // Define the thickness of the line
        int lineThickness = 1;
        // Draw the line on the image
        cv::line(combinedImage, startPoint, endPoint, lineColor, lineThickness);
    }

    return combinedImage;
}

cv::Mat getIntrinsicsFromSensor(VirtualSensor& sensor)
{
    // Get the intrinsics from the sensor
    Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();

    // Extract the intrinsic parameters
    float fX = depthIntrinsics(0, 0);
    float fY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);

    // Construct the K matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << fX, 0, cX,
                                           0, fY, cY,
                                           0, 0, 1);

    return K;
}

void savePointsToFile(const std::vector<cv::Point3f>& points3D, const std::string& filename)
{
    std::ofstream outFile(filename);
    if (!outFile)
    {
        std::cerr << "Error: Could not open file for writing." << std::endl;
        return;
    }

    for (const auto& point : points3D)
    {
        outFile << point.x << " " << point.y << " " << point.z << std::endl;
    }

    outFile.close();
}