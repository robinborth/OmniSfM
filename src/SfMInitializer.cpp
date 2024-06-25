#include "SfMInitializer.h"

#include "Eigen.h"
#include <opencv2/flann.hpp>
#include <opencv2/core/eigen.hpp>
#include <limits>

SfMInitializer::SfMInitializer() {}

bool SfMInitializer::estimateInitialPose(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, const Eigen::Matrix3f &K, cv::Mat &R, cv::Mat &t)
{

    cv::Mat k; // HACK convert to cv2
    cv::eigen2cv(K, k);
    cv::Mat E = cv::findEssentialMat(pts1, pts2, k, cv::RANSAC, 0.999, 1.0);
    if (E.empty())
    {
        std::cerr << "Essential matrix estimation failed." << std::endl;
        return false;
    }

    // Recover pose from the Essential matrix
    int inliers = cv::recoverPose(E, pts1, pts2, k, R, t);
    std::cout << "Recovered pose with " << inliers << " inliers." << std::endl;
    return inliers > 0;
}

std::vector<ColoredPoint3f> SfMInitializer::triangulatePointsWithColor(const std::vector<cv::Point2f> &pts1,
                                                                       const std::vector<cv::Point2f> &pts2,
                                                                       const cv::Mat &R, const cv::Mat &t,
                                                                       const Eigen::Matrix3f &K,
                                                                       const std::vector<cv::Vec3b> &colors1,
                                                                       const std::vector<cv::Vec3b> &colors2)
{

    // Compute projection matrices for both cameras
    cv::Mat k; // HACK convert to cv2
    cv::eigen2cv(K, k);
    k.convertTo(k, CV_64F);
    cv::Mat P1 = k * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2 = k * (cv::Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
                      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
                      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));

    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);
    std::vector<ColoredPoint3f> coloredPoints3D;
    for (int i = 0; i < points4D.cols; ++i)
    {
        cv::Vec4d point = points4D.col(i);
        point /= point[3]; // Convert from homogeneous to 3D coordinates
        cv::Vec3b color = cv::Vec3b(
            (colors1[i][0] + colors2[i][0]) / 2,
            (colors1[i][1] + colors2[i][1]) / 2,
            (colors1[i][2] + colors2[i][2]) / 2);
        coloredPoints3D.emplace_back(ColoredPoint3f{cv::Point3f(point[0], point[1], point[2]), color});
    }
    return coloredPoints3D;
}

bool SfMInitializer::estimatePosePnP(const std::vector<cv::Point3f> &objectPoints, const std::vector<cv::Point2f> &imagePoints, const Eigen::Matrix3f &K, cv::Mat &R, cv::Mat &t)
{
    cv::Mat k; // HACK convert to cv2
    cv::eigen2cv(K, k);
    cv::Mat rvec;
    bool success = cv::solvePnPRansac(objectPoints, imagePoints, k, cv::noArray(), rvec, t, true, 100, 8.0, 0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);
    if (!success)
    {
        std::cerr << "Pose estimation PnP failed." << std::endl;
        return false;
    }

    cv::Rodrigues(rvec, R); // Convert rotation vector to matrix
    return true;
}

Eigen::Matrix4f SfMInitializer::combineRotationAndTranslationIntoMatrix(const cv::Mat &R, const cv::Mat &t)
{
    Eigen::Matrix3f eR;
    Eigen::Vector3f et;
    cv::cv2eigen(R, eR);
    cv::cv2eigen(t, et);
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity(); // Initialize with identity
    pose.block<3, 3>(0, 0) = eR;                        // rotation matrix
    pose.block<3, 1>(0, 3) = et;                        //  translation vector

    return pose;
}

void SfMInitializer::runSfM(std::vector<Image> &images, std::vector<std::vector<Match>> &allMatches)
{
    if (allMatches.size() > 0 && allMatches[0].size() >= 60)
    {
        std::vector<cv::Point2f> pts1, pts2;
        std::vector<cv::Vec3b> colors1, colors2;
        for (const auto &match : allMatches[0])
        {
            cv::Point2f pt1 = images[0].keypoints[match.sourceKeypointId].pt;
            cv::Point2f pt2 = images[1].keypoints[match.targetKeyopintId].pt;
            pts1.push_back(pt1);
            pts2.push_back(pt2);
            colors1.push_back(images[0].rgb.at<cv::Vec3b>((int)pt1.y, (int)pt1.x));
            colors2.push_back(images[1].rgb.at<cv::Vec3b>((int)pt2.y, (int)pt2.x));
        }

        Eigen::Matrix3f &K = images[0].K;
        std::cout << "==> Estimate initial pose ..." << std::endl;

        // Estimate initial pose
        cv::Mat R, t;
        if (estimateInitialPose(pts1, pts2, K, R, t))
        {
            std::cout << "==> Initial pose estimation successful." << std::endl;
            this->cameraPoses.push_back(combineRotationAndTranslationIntoMatrix(R, t)); // Save initial pose
            // Triangulate points and store them
            this->points3D = triangulatePointsWithColor(pts1, pts2, R, t, K, colors1, colors2);
            std::cout << "==> Triangulated " << points3D.size() << " points." << std::endl;
            // Incremental pose estimation for other images
            for (size_t i = 2; i < images.size(); ++i)
            {
                std::vector<cv::Point2f> imagePoints;
                std::vector<cv::Point3f> objectPoints;
                for (const auto &match : allMatches[i - 1])
                {
                    if (match.sourceKeypointId < points3D.size())
                    {
                        imagePoints.push_back(images[i].keypoints[match.targetKeyopintId].pt);
                        objectPoints.push_back(points3D[match.sourceKeypointId].point);
                    }
                }

                cv::Mat Ri, ti;
                if (estimatePosePnP(objectPoints, imagePoints, images[i].K, Ri, ti))
                {
                    std::cout << "Pose estimation successful for image " << i << "." << std::endl;
                    cameraPoses.push_back(combineRotationAndTranslationIntoMatrix(Ri, ti));
                }
                else
                {
                    std::cout << "Pose estimation failed for image " << i << "." << std::endl;
                }
            }

            // bundle adjustment
        }
    }
}

const std::vector<ColoredPoint3f> &SfMInitializer::getPoints3D() const { return points3D; }
const std::vector<Eigen::Matrix4f> &SfMInitializer::getCameraPoses() const { return cameraPoses; }