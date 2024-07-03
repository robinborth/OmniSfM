#include "CorrespondenceSearch.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

using namespace std;

std::vector<cv::DMatch> CorrespondenceSearch::queryMatches(const Image& sourceImage, const Image& targetImage)
{
    // Size of the outer vector is the number of keypoints in the source image
    // Each element of the outer vector is another vector containing k (cv::DMatch objects)
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
    matcher->knnMatch(sourceImage.descriptors, targetImage.descriptors, matches, 2);
    vector<cv::DMatch> good_matches;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < 0.5f * matches[i][1].distance) 
        {
            good_matches.push_back(matches[i][0]);
        }
    }
    cout << "==> Found " << good_matches.size() << " good matches ..." << endl;
    return good_matches;
}

ImagePairMatches CorrespondenceSearch::queryCorrespondences(const std::vector<Image>& images)
{
    ImagePairMatches allMatches;

    for (size_t i = 0; i < images.size() - 1; ++i)
    {
        for (size_t j = i + 1; j < images.size(); ++j)
        {
            std::cout << "==> Finding correspondences between images " << i << " and " << j << " ..." << std::endl;
            vector<cv::DMatch> matches = queryMatches(images[i], images[j]);

            vector<cv::DMatch> inlierMatches = this->filterMatchesWithRANSAC(images[i], images[j], matches);

            // Only keep pairs with at least 60 matches
            if (inlierMatches.size() >= 60)
            {
                printf("==> Found %lu inlier matches between images %lu and %lu\n", inlierMatches.size(), i, j);
                allMatches[std::make_pair(i, j)] = matches;
            }
        }
    }
    return allMatches;
}

std::vector<cv::DMatch> CorrespondenceSearch::filterMatchesWithRANSAC(const Image& sImg, const Image& tImg, const std::vector<cv::DMatch>& matches)
{
    // Convert keypoints to Point2f
    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;

    for (const auto& match : matches)
    {
        srcPoints.push_back(sImg.keypoints[match.queryIdx].pt);
        dstPoints.push_back(tImg.keypoints[match.trainIdx].pt);
    }

    // Check if there are enough points for the 8-point algorithm
    if (srcPoints.size() < 8 || dstPoints.size() < 8) {
        std::cerr << "Not enough points to use the 8-point algorithm" << std::endl;
        return {}; // Return an empty vector if not enough points
    }

    // Use RANSAC to find the fundamental matrix
    std::vector<uchar> inliersMask(srcPoints.size(), 0);
    cv::Mat fundamentalMatrix = cv::findFundamentalMat(srcPoints, dstPoints, cv::FM_RANSAC, 1, 0.999, inliersMask);

    // Filter matches based on inliers mask
    std::vector<cv::DMatch> inlierMatches;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (inliersMask[i])
            inlierMatches.push_back(matches[i]);
    }

    return inlierMatches;
}

cv::Mat CorrespondenceSearch::visualizeCorrespondences(const Image& sImg, const Image& tImg, const std::vector<cv::DMatch>& matches)
{
    std::vector<cv::DMatch> cvMatches;
    for (const auto& match : matches)
    {
        cvMatches.emplace_back(match.queryIdx, match.trainIdx, match.distance);
    }

    cv::Mat imgMatches;
    cv::drawMatches(sImg.rgb, sImg.keypoints, tImg.rgb, tImg.keypoints, cvMatches, imgMatches);
    return imgMatches;
}
