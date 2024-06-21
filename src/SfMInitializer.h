#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <vector>
#include <iostream>
#include <limits>

#include "ImageStorage.h"
#include "CorrespondenceSearch.h"

class SfMInitializer
{
public:
    SfMInitializer() {}

    bool estimateInitialPose(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, const cv::Mat& K, cv::Mat& R, cv::Mat& t)
    {
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0);
        if (E.empty())
        {
            std::cerr << "Essential matrix estimation failed." << std::endl;
            return false;
        }

        // Recover pose from the Essential matrix
        int inliers = cv::recoverPose(E, pts1, pts2, K, R, t);
        std::cout << "Recovered pose with " << inliers << " inliers." << std::endl;
        return inliers > 0;
    }

    std::vector<cv::Point3f> triangulatePoints(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, const cv::Mat& R, const cv::Mat& t, const cv::Mat& K) {
        // Compute projection matrices for both cameras
        cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);
        cv::Mat P2 = K * (cv::Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
                                                          R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
                                                          R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));

        cv::Mat points4D;
        cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

        std::vector<cv::Point3f> points3D;
        for (int i = 0; i < points4D.cols; ++i) {
            cv::Vec4d point = points4D.col(i);
            point /= point[3];  // Convert from homogeneous to 3D coordinates
            points3D.emplace_back(point[0], point[1], point[2]);
        }
        return points3D;
    }

    bool estimatePosePnP(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& K, cv::Mat& R, cv::Mat& t) {
        cv::Mat rvec;
        bool success = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::noArray(), rvec, t, true, 100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
        if (!success) {
            std::cerr << "Pose estimation PnP failed." << std::endl;
            return false;
        }

        cv::Rodrigues(rvec, R);  // Convert rotation vector to matrix
        return true;
    }

    void runSfM(std::vector<Image>& images, std::vector<std::vector<Match>>& allMatches)
    {
        if (allMatches.size() > 0 && allMatches[0].size() >= 60)
        {
            std::vector<cv::Point2f> pts1, pts2;
            for (const auto& match : allMatches[0])
            {
                pts1.push_back(images[0].keypoints[match.sourceKeypointId].pt);
                pts2.push_back(images[1].keypoints[match.targetKeyopintId].pt);
            }

            cv::Mat& K = images[0].K;
            std::cout << "==> Estimate initial pose ..." << std::endl;

            // Estimate initial pose
            cv::Mat R, t;
            if (estimateInitialPose(pts1, pts2, K, R, t))
            {
                std::cout << "==> Initial pose estimation successful." << std::endl;
                // Triangulate points and store them
                points3D = triangulatePoints(pts1, pts2, R, t, K);
                std::cout << "==> Triangulated " << points3D.size() << " points." << std::endl;
                // Incremental pose estimation for other images
                for (size_t i = 2; i < images.size(); ++i) {
                    std::vector<cv::Point2f> imagePoints;
                    std::vector<cv::Point3f> objectPoints;
                    for (const auto& match : allMatches[i - 1]) {
                        if (match.sourceKeypointId < points3D.size()) {
                            imagePoints.push_back(images[i].keypoints[match.targetKeyopintId].pt);
                            objectPoints.push_back(points3D[match.sourceKeypointId]);
                        }
                    }

                    cv::Mat Ri, ti;
                    if (estimatePosePnP(objectPoints, imagePoints, images[i].K, Ri, ti)) {
                        std::cout << "Pose estimation successful for image " << i << "." << std::endl;
                    } else {
                        std::cout << "Pose estimation failed for image " << i << "." << std::endl;
                    }
                }

                // bundle adjustment
            }
        }
    }

    const std::vector<cv::Point3f>& getPoints3D() const { return points3D; }

private:
    std::vector<cv::Point3f> points3D; // Store the 3D points
};