#include "CorrespondenceSearch.h"
#include <opencv2/calib3d.hpp>

void CorrespondenceSearch::buildIndex(const Image& targetImage)
{
    this->flannIndex = cv::makePtr<cv::flann::Index>(targetImage.descriptors, cv::flann::KDTreeIndexParams(5));
    this->targetDescriptors = targetImage.descriptors;
}

Match CorrespondenceSearch::getClosestPoint(const cv::Mat& sourceDescriptor, float loweRatio)
{
    std::vector<int> indices(2); // Find the 2 nearest neighbors
    std::vector<float> dists(2); // Their distances
    this->flannIndex->knnSearch(sourceDescriptor, indices, dists, 2, cv::flann::SearchParams(8)); // Using 2 neighbors for ratio test

    Match match;
    if (dists[0] < loweRatio * dists[1]) // Lowe's ratio test
    {
        match.targetKeyopintId = indices[0];
        match.weight = dists[0];
    }
    else
    {
        match.targetKeyopintId = -1;
        match.weight = std::numeric_limits<float>::max();
    }
    return match;
}

std::vector<cv::Mat> CorrespondenceSearch::matToVector(const cv::Mat& mat)
{
    std::vector<cv::Mat> vec(mat.rows);
    for (int i = 0; i < mat.rows; ++i)
    {
        vec[i] = mat.row(i);
    }
    return vec;
}

std::vector<Match> CorrespondenceSearch::queryMatches(const Image& sourceImage, const Image& targetImage)
{
    // Build the index for the target image
    buildIndex(targetImage);

    // Prepare the source points from the source image
    auto sourcePoints = matToVector(sourceImage.descriptors);

    // Find the matches and return them
    std::vector<Match> matches;
    for (size_t i = 0; i < sourcePoints.size(); ++i)
    {
        Match match = getClosestPoint(sourcePoints[i], 0.02f);
        if (match.targetKeyopintId != -1)
        {
            match.sourceKeypointId = i;
            match.targetImageId = targetImage.id;
            match.sourceImageId = sourceImage.id;
            matches.push_back(match);
        }
    }
    std::cout << "==> Before Filtering Found " << matches.size() << " matches ..." << std::endl;
    return matches;
}

std::vector<std::vector<Match>> CorrespondenceSearch::queryCorrespondences(const std::vector<Image>& images)
{
    std::vector<std::vector<Match>> allMatches(images.size() - 1); // Correctly sized to store pairwise matches

    for (size_t i = 0; i < images.size() - 1; ++i)
    {
        for (size_t j = i + 1; j < images.size(); ++j)
        {
            std::cout << "==> Finding correspondences between images " << i << " and " << j << " ..." << std::endl;
            auto matches = queryMatches(images[i], images[j]);
            auto inlierMatches = this->filterMatchesWithRANSAC(images[i], images[j], matches);

            // Only keep pairs with at least 60 matches
            if (inlierMatches.size() >= 60)
            {
                printf("==> Found %lu inlier matches between images %lu and %lu\n", inlierMatches.size(), i, j);
                allMatches[i].insert(allMatches[i].end(), inlierMatches.begin(), inlierMatches.end());
            }
        }
    }
    return allMatches;
}

std::vector<Match> CorrespondenceSearch::filterMatchesWithRANSAC(const Image& sImg, const Image& tImg, const std::vector<Match>& matches)
{
    // Convert keypoints to Point2f
    std::vector<cv::Point2f> srcPoints;
    std::vector<cv::Point2f> dstPoints;

    for (const auto& match : matches)
    {
        srcPoints.push_back(sImg.keypoints[match.sourceKeypointId].pt);
        dstPoints.push_back(tImg.keypoints[match.targetKeyopintId].pt);
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
    std::vector<Match> inlierMatches;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (inliersMask[i]) {
            inlierMatches.push_back(matches[i]);
        }
    }

    return inlierMatches;
}

cv::Mat CorrespondenceSearch::visualizeCorrespondences(const Image& sImg, const Image& tImg, const std::vector<Match>& matches)
{
    std::vector<cv::DMatch> cvMatches;
    for (const auto& match : matches)
    {
        cvMatches.emplace_back(match.sourceKeypointId, match.targetKeyopintId, match.weight);
    }

    cv::Mat imgMatches;
    cv::drawMatches(sImg.rgb, sImg.keypoints, tImg.rgb, tImg.keypoints, cvMatches, imgMatches);
    return imgMatches;
}
