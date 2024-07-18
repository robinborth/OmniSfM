#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include "Definitions.h"
#include "ImageStorage.h"
#include "SfmGraph.h"

class SfMInitializer
{
public:
    SfMInitializer(ImageStorage &imageStorage, SfMGraph &graph);

    std::vector<Eigen::Vector4f> vertex2Camera(size_t imgIdx, std::vector<Vertex> points3D);
    const std::vector<Vertex> &getPoints3D() const;
    void solveDepthMaps(size_t imgIdx, std::vector<cv::Point2f> points2D, std::vector<Eigen::Vector4f> points3D);
    cv::Mat getIntrinsic();
    cv::Mat world2Image(Image img);
    const std::vector<Eigen::Matrix4f> getCameraPoses() const;
    Eigen::Matrix4f combineRotationAndTranslationIntoMatrix(const cv::Mat &R, const cv::Mat &t);
    void twoViewSfm(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2);
    void multiViewSfm(const ImagePairMatches &allMatches, int newImageId);
    void addSfM(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2);

private:
    bool estimateInitialPose(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, cv::Mat &R, cv::Mat &t);
    std::vector<Vertex> triangulatePointsWithColor(const std::vector<cv::Point2f> &pts1,
                                                   const std::vector<cv::Point2f> &pts2,
                                                   const std::vector<cv::Vec3b> &colors1,
                                                   const std::vector<cv::Vec3b> &colors2,
                                                   const cv::Mat &P1,
                                                   const cv::Mat &P2);
    std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, std::vector<cv::Vec3b>, std::vector<cv::Vec3b>> extractMatchedPoints(const std::vector<cv::DMatch> &matches, size_t idx1, size_t idx2);
    bool refineCameraPoseWithPnP(const std::vector<cv::Point3f> &objectPoints,
                                 const std::vector<cv::Point2f> &imagePoints,
                                 size_t cameraId,
                                 cv::Mat &R,
                                 cv::Mat &t);
    std::vector<cv::Point3f> convertVerticesToCvPoint3f(const std::vector<Vertex> &vertices);
    void updateGraph(const std::vector<Vertex> &points3D, const std::vector<cv::DMatch> &matches, Image *img1, Image *img2);

    std::vector<Vertex> points3D;
    ImageStorage &imageStorage;
    std::vector<Eigen::Matrix4f> poses;
    SfMGraph &graph;
};
