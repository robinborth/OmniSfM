#pragma once

#include <string>
#include <vector>
#include "Definitions.h"
#include "Settings.h"
#include "Visualization.h"
#include <opencv2/opencv.hpp>

class ImageStorage
{
public:
    ImageStorage(const Settings &settings);

    void loadImages();
    void useEveryNthImage(int n);

    static Eigen::Matrix4f convertToEigen(const cv::Mat &R, const cv::Mat &t);
    static void createPointCloudFromImage(const Image &img, Visualization &visualization);
    void processImagesForPointCloud();

    void detectKeypoints();
    void drawKeypoints(int64_t id, std::string outputPath);

    Image *findImage(int64_t id);
    Eigen::Matrix3f getIntrinsics() const;
    void updatePose(int64_t id, const Eigen::Matrix4f &pose);
    void updateScale(int64_t id, float scale);
    void updateShift(int64_t id, float shift);
    int getNumImages() const;
    Eigen::Matrix4f getPose(int64_t id);
    std::vector<Image> images;

private:
    bool readIntrinsics();

    bool readExtrinsics();

    bool readFileList(const std::string &type, std::vector<std::string> &filenames, std::vector<int64_t> &ids);

    static bool checkLoadImage(cv::Mat &image);

    bool loadImages(const std::string &type, std::vector<std::string> &filenames, std::vector<int64_t> &ids);

    std::string datasetDir;
    cv::Ptr<cv::SIFT> sift;
    std::vector<std::string> filenameDepthImages;
    std::vector<int64_t> idDepthImages;
    std::vector<std::string> filenameRGBImages;
    std::vector<int64_t> idRGBImages;
};