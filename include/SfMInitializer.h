#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include "Definitions.h"

class SfMInitializer
{
public:
    SfMInitializer(std::vector<Image> &images);

    void runSfM(ImagePairMatches &allMatches);
    Eigen::Matrix4f debugRunSfm(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2);
    const std::vector<Vertex> &getPoints3D() const;
    const std::map<int, Eigen::Matrix4f> &getCameraPoses() const;
    Eigen::Matrix4f combineRotationAndTranslationIntoMatrix(const cv::Mat &R, const cv::Mat &t);

private:
    bool estimateInitialPose(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, cv::Mat &R, cv::Mat &t);
    std::vector<Vertex> triangulatePointsWithColor(const std::vector<cv::Point2f> &pts1,
                                                           const std::vector<cv::Point2f> &pts2,
                                                           const cv::Mat &R, const cv::Mat &t,
                                                           const std::vector<cv::Vec3b> &colors1,
                                                           const std::vector<cv::Vec3b> &colors2);
    void extractMatchedPoints(const std::vector<cv::DMatch>& matches, size_t idx1, size_t idx2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<cv::Vec3b>& colors1, std::vector<cv::Vec3b>& colors2);
    Image* findImageById(int imageId);
    bool refineCameraPoseWithPnP(const std::vector<Vertex>& objectPoints,
                                             const std::vector<cv::Point2f>& imagePoints,
                                             size_t cameraId,
                                             const cv::Mat& R,
                                             const cv::Mat& t);
    std::vector<cv::Point3f> convertVerticesToCvPoint3f(const std::vector<Vertex>& vertices);

    std::vector<Vertex> points3D;
    std::map<int, Eigen::Matrix4f> cameraPoses; // Store the camera poses key = image id, value = camera pose
    std::vector<Image> images;
    Eigen::Matrix3f K;
};
