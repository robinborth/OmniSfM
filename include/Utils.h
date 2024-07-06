#pragma once

#include "Eigen.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "Definitions.h"

Eigen::Vector4f image2camera(float x, float y, float depth, Eigen::Matrix3f &K)
{
    float fX = K(0, 0);
    float fY = K(1, 1);
    float cX = K(0, 2);
    float cY = K(1, 2);
    float x_ = (x - cX) * depth / fX;
    float y_ = (y - cY) * depth / fY;
    return Eigen::Vector4f(x_, y_, depth, 1.0);
}

Eigen::Matrix4f camera2worldMatrix(const cv::Mat &R, const cv::Mat &t)
{
    Eigen::Matrix3f eR;
    Eigen::Vector3f et;
    cv::cv2eigen(R, eR);
    cv::cv2eigen(t, et);
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = eR;
    pose.block<3, 1>(0, 3) = et;
    return pose;
}

std::vector<Vertex> extractPointCloud(Image &img)
{
    auto P = camera2worldMatrix(img.R, img.t);
    std::vector<Vertex> points;
    for (int y = 0; y < img.depth.rows(); ++y)
    {
        for (int x = 0; x < img.depth.cols(); ++x)
        {
            // extract the depth value and transform
            float rawDepth = img.depth(y, x);
            float depth = rawDepth * img.w + img.q;

            // image to camera transformation
            Eigen::Vector4f c_point = image2camera(x, y, depth, img.K);
            Eigen::Vector4f w_point = P * c_point;

            // update the colord point but as RGB
            auto pixelBGR = img.rgb.at<cv::Vec3b>(y, x);
            Vector4uc _color = {
                pixelBGR[2],  // Red
                pixelBGR[1],  // Green
                pixelBGR[0],  // Blue
                255           // Alpha channel set to maximum
            };
            Vertex vertex{w_point, _color};

            points.push_back(vertex);
        }
    }
    return points;
}
