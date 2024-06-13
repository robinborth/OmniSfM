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
    SfMInitializer(const cv::Mat& K) : K(K) {}

    bool estimateInitialPose(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, cv::Mat& R, cv::Mat& t)
    {
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K);
        if (E.empty())
        {
            std::cerr << "Essential matrix estimation failed." << std::endl;
            return false;
        }

        cv::recoverPose(E, pts1, pts2, K, R, t);
        return true;
    }

    std::vector<cv::Point3f> triangulatePoints(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, const cv::Mat& R1, const cv::Mat& t1, const cv::Mat& R2, const cv::Mat& t2)
    {
        cv::Mat proj1, proj2;
        cv::hconcat(R1, t1, proj1);
        cv::hconcat(R2, t2, proj2);

        proj1 = K * proj1;
        proj2 = K * proj2;

        cv::Mat points4D;
        cv::triangulatePoints(proj1, proj2, pts1, pts2, points4D);

        std::vector<cv::Point3f> points3D;
        for (int i = 0; i < points4D.cols; ++i)
        {
            cv::Mat col = points4D.col(i);
            col /= col.at<float>(3); // Normalize homogeneous coordinates
            points3D.emplace_back(col.at<float>(0), col.at<float>(1), col.at<float>(2));
        }
        return points3D;
    }

    bool estimatePosePnP(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, cv::Mat& R, cv::Mat& t)
    {
        cv::Mat rvec;
        bool success = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(), rvec, t);
        if (!success)
        {
            std::cerr << "Pose estimation PnP failed." << std::endl;
            return false;
        }

        cv::Rodrigues(rvec, R);
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

            // Estimate initial pose
            cv::Mat R, t;
            if (estimateInitialPose(pts1, pts2, R, t))
            {
                // Triangulate points and store them
                points3D = triangulatePoints(pts1, pts2, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), R, t);

                // Incremental pose estimation for other images
                for (size_t i = 2; i < images.size(); ++i)
                {
                    std::vector<cv::Point2f> imagePoints;
                    std::vector<cv::Point3f> objectPoints;
                    for (const auto& match : allMatches[i - 1])
                    {
                        imagePoints.push_back(images[i].keypoints[match.targetKeyopintId].pt);
                        objectPoints.push_back(points3D[match.sourceKeypointId]);
                    }

                    // Estimate pose for the new image
                    cv::Mat Ri, ti;
                    if (estimatePosePnP(objectPoints, imagePoints, Ri, ti))
                    {
                        // Add new 3D points using triangulation
                        std::vector<cv::Point3f> newPoints3D = triangulatePoints(pts1, pts2, R, t, Ri, ti);
                        points3D.insert(points3D.end(), newPoints3D.begin(), newPoints3D.end());

                        R = Ri.clone();
                        t = ti.clone();
                    }
                }

                // bundle adjustment
            }
        }
    }

    const std::vector<cv::Point3f>& getPoints3D() const { return points3D; }

private:
    cv::Mat K; // Camera intrinsic matrix
    std::vector<cv::Point3f> points3D; // Store the 3D points
};