#pragma once

#include "Eigen.h"
#include "Definitions.h"
#include <opencv2/flann.hpp>
#include <opencv2/highgui.hpp>


class CorrespondenceSearch
{
public:

    std::vector<cv::DMatch> queryMatches(const Image& sourceImage, const Image& targetImage);

	ImagePairMatches queryCorrespondences(const std::vector<Image>& images);

    std::vector<cv::DMatch> filterMatchesWithRANSAC(const Image& sImg, const Image& tImg, const std::vector<cv::DMatch>& matches);

	cv::Mat visualizeCorrespondences(const Image& sImg, const Image& tImg, const std::vector<cv::DMatch>& matches);

private:
    cv::Ptr<cv::flann::Index> flannIndex;
};