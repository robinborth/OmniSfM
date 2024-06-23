#pragma once

#include "Eigen.h"
#include "Definitions.h"
#include <opencv2/flann.hpp>
#include <opencv2/highgui.hpp>


class CorrespondenceSearch
{
public:
    void buildIndex(const Image& targetImage);

    Match getClosestPoint(const cv::Mat& sourceDescriptor, float loweRatio);

    std::vector<cv::Mat> matToVector(const cv::Mat& mat);

    std::vector<Match> queryMatches(const Image& sourceImage, const Image& targetImage);

	std::vector<std::vector<Match>> queryCorrespondences(const std::vector<Image>& images);

    std::vector<Match> filterMatchesWithRANSAC(const Image& sImg, const Image& tImg, const std::vector<Match>& matches);

	cv::Mat visualizeCorrespondences(const Image& sImg, const Image& tImg, const std::vector<Match>& matches);

private:
    cv::Ptr<cv::flann::Index> flannIndex;
    cv::Mat targetDescriptors;
};