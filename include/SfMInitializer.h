#pragma once
#include <vector>
#include <opencv2/opencv.hpp>
#include "Definitions.h"

class SfMInitializer
{
public:
    SfMInitializer();

    void runSfM(std::vector<Image>& images, std::vector<std::vector<Match>>& allMatches);
    const std::vector<ColoredPoint3f>& getPoints3D() const;
    const std::vector<Eigen::Matrix4f>& getCameraPoses() const;

private:
    bool estimateInitialPose(const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, const cv::Mat& K, cv::Mat& R, cv::Mat& t);
    std::vector<ColoredPoint3f> triangulatePointsWithColor(const std::vector<cv::Point2f>& pts1,                                                         
                                                        const std::vector<cv::Point2f>& pts2, 
                                                        const cv::Mat& R, const cv::Mat& t, 
                                                        const cv::Mat& K, 
                                                        const std::vector<cv::Vec3b>& colors1,
                                                        const std::vector<cv::Vec3b>& colors2);
    bool estimatePosePnP(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints, const cv::Mat& K, cv::Mat& R, cv::Mat& t);
    Eigen::Matrix4f combineRotationAndTranslationIntoMatrix(const cv::Mat& R, const cv::Mat& t);

    std::vector<ColoredPoint3f> points3D;
    std::vector<Eigen::Matrix4f> cameraPoses;  // Store the camera poses
};
