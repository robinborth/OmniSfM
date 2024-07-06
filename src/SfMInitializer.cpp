#include "SfMInitializer.h"

#include <opencv2/flann.hpp>
#include <opencv2/core/eigen.hpp>
#include <limits>
#include "Eigen.h"

SfMInitializer::SfMInitializer(std::vector<Image> &images) 
{
    this->images = images;
    this->K = images[0].K;
}

Image* SfMInitializer::findImageById(int imageId) 
{
    auto it = std::find_if(this->images.begin(), this->images.end(), [imageId](const Image& img) {
        return img.id == imageId;
    });

    if (it != this->images.end()) {
        return &(*it);
    }
    return nullptr;
}

std::vector<cv::Point3f> SfMInitializer::convertVerticesToCvPoint3f(const std::vector<Vertex>& vertices) 
{
    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(vertices.size());
    for (const auto& vertex : vertices) {
        objectPoints.emplace_back(cv::Point3f(vertex.position(0), vertex.position(1), vertex.position(2)));
    }
    return objectPoints;
}

bool SfMInitializer::estimateInitialPose(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, cv::Mat &R, cv::Mat &t)
{
    cv::Mat k; // HACK convert to cv2
    cv::eigen2cv(this->K, k);
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

std::vector<Vertex> SfMInitializer::triangulatePointsWithColor(const std::vector<cv::Point2f> &pts1,
                                                                       const std::vector<cv::Point2f> &pts2,
                                                                       const cv::Mat &R, const cv::Mat &t,
                                                                       const std::vector<cv::Vec3b> &colors1,
                                                                       const std::vector<cv::Vec3b> &colors2)
{

    // Compute projection matrices for both cameras
    cv::Mat k; // HACK convert to cv2
    cv::eigen2cv(this->K, k);
    k.convertTo(k, CV_64F);
    cv::Mat P1 = k * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2 = k * (cv::Mat_<double>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0),
                      R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1),
                      R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2));

    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);
    std::vector<Vertex> vertices;
    for (int i = 0; i < points4D.cols; ++i) {
        cv::Vec4d point = points4D.col(i);
        point /= point[3]; // Normalize to convert from homogeneous to Cartesian coordinates

        // Average the colors from both images
        Vector4uc color(
            (colors1[i][0] + colors2[i][0]) / 2,
            (colors1[i][1] + colors2[i][1]) / 2,
            (colors1[i][2] + colors2[i][2]) / 2,
            255  // Alpha channel set to maximum
        );
        Vertex vertex{{point[0], point[1], point[2], 1.0f}, color};
        this->points3D.push_back(vertex);
        vertices.push_back(vertex);
    }

    return vertices;
}

bool SfMInitializer::refineCameraPoseWithPnP(const std::vector<Vertex>& objectPoints,
                                             const std::vector<cv::Point2f>& imagePoints,
                                             size_t cameraId,
                                             const cv::Mat& R,
                                             const cv::Mat& t) {
    // Prepare rotation and translation vectors for the solvePnP function
    cv::Mat rvec, tvec, k;
    cv::Rodrigues(R, rvec);
    cv::eigen2cv(this->K, k);


    // Execute the solvePnP function to refine the pose
    std::vector<cv::Point3f> objectPointsCv = convertVerticesToCvPoint3f(objectPoints);
    bool success = cv::solvePnP(objectPointsCv, imagePoints, k, cv::noArray(), rvec, t, true, cv::SOLVEPNP_ITERATIVE);

    if (success) {
        cv::Rodrigues(rvec, R);  // Convert rotation vector back to rotation matrix
        std::cout << "Pose refinement successful for camera " << cameraId << std::endl;

        Eigen::Matrix4f refinedPose = combineRotationAndTranslationIntoMatrix(R, t);
        this->cameraPoses[cameraId] = refinedPose;
    }

    return success;
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

void SfMInitializer::extractMatchedPoints(const std::vector<cv::DMatch>& matches,
                          size_t idx1, size_t idx2,
                          std::vector<cv::Point2f>& pts1,
                          std::vector<cv::Point2f>& pts2,
                          std::vector<cv::Vec3b>& colors1,
                          std::vector<cv::Vec3b>& colors2) 
{
    std::cout << "Extracting Matched Points for id " << idx1 << "and id " << idx2 << std::endl; 
    Image* img1 = findImageById(idx1);
    Image* img2 = findImageById(idx2);
    if (!img1 || !img2) {
        std::cerr << "Image not found." << std::endl;
        return;
    }
    for (const auto& match : matches) {
        cv::Point2f pt1 = img1->keypoints[match.queryIdx].pt;
        cv::Point2f pt2 = img2->keypoints[match.trainIdx].pt;
        pts1.push_back(pt1);
        pts2.push_back(pt2);
        colors1.push_back(img1->rgb.at<cv::Vec3b>((int)pt1.y, (int)pt1.x));
        colors2.push_back(img2->rgb.at<cv::Vec3b>((int)pt2.y, (int)pt2.x));
    }
}

void SfMInitializer::runSfM(ImagePairMatches &allMatches)
{
    if (allMatches.empty()) {
        std::cout << "No matches to process." << std::endl;
        return;
    }
    const auto &firstPair = allMatches.begin()->first;
    this->cameraPoses[firstPair.first] = Eigen::Matrix4f::Identity();  // Set the first camera's pose as identity


    for (const auto &pair : allMatches) 
    {
        int sourceImgId = pair.first.first;
        int targetImdId = pair.first.second;
        std::cout << sourceImgId << " " << pair.first.second << " " << pair.second.size() << std::endl;
        if (pair.second.size() < 60) 
        {
            std::cout << "Not enough matches for image pair (" << sourceImgId << ", " << targetImdId << ")." << std::endl;
            continue;
        }
        std::vector<cv::Point2f> pts1, pts2;
        std::vector<cv::Vec3b> colors1, colors2;
        extractMatchedPoints(pair.second, sourceImgId, targetImdId, pts1, pts2, colors1, colors2);

        cv::Mat R, t;
        if (estimateInitialPose(pts1, pts2, R, t)) 
        {
            /*
                estimateInitialPose function returns the relative pose from camera 1 to camera 2
                Pose of the camera2  T_2 = T_1.T_12
            */
            Eigen::Matrix4f pose = combineRotationAndTranslationIntoMatrix(R, t);
            cameraPoses[targetImdId] = cameraPoses[sourceImgId] * pose;


            std::vector<Vertex> _points3D = triangulatePointsWithColor(pts1, pts2, R, t, colors1, colors2);

            // Use PnP to refine the pose of the new camera
            refineCameraPoseWithPnP(_points3D, pts2, targetImdId, R, t);
            std::cout << "Estimated pose for image pair (" << sourceImgId << ", " << targetImdId << ")." << std::endl;
        } else {
            std::cout << "Failed to estimate pose for image pair (" << sourceImgId << ", " << targetImdId << ")." << std::endl;
        }
    }
}

Eigen::Matrix4f SfMInitializer::debugRunSfm(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2)
{
    if (imgId1 >= this->images.size() || imgId2 >= this->images.size() || imgId1 == imgId2) {
        std::cerr << "Invalid image indices provided. Indices must be within the range of the image vector and not equal." << std::endl;
    }

    // Check if there are enough matches to proceed
    if (matchesForPair.size() < 60) {
        std::cout << "Not enough matches to estimate a reliable pose (" << matchesForPair.size() << " matches found)." << std::endl;
    }

    // Extract points and colors for the matched keypoints
    std::vector<cv::Point2f> pts1, pts2;
    std::vector<cv::Vec3b> colors1, colors2;
    extractMatchedPoints(matchesForPair, imgId1, imgId2, pts1, pts2, colors1, colors2);

    std::cout << "==> Extracted " << pts1.size() << " points for pose estimation." << std::endl;
    std::cout << "==> Extracted " << pts2.size() << " points for pose estimation." << std::endl;

    // Assuming the intrinsic matrix K is the same for both images
    Eigen::Matrix3f &K = this->images[imgId1].K;
    std::cout << "==> Estimate initial pose ..." << std::endl;

    // Estimate initial pose using the extracted points
    cv::Mat R, t;
    if (estimateInitialPose(pts1, pts2, R, t)) {
        std::cout << "Initial pose estimation successful." << std::endl;
        return combineRotationAndTranslationIntoMatrix(R, t);
    } else {
        std::cout << "Failed to estimate initial pose." << std::endl;
        return Eigen::Matrix4f::Identity();
    }
}

const std::vector<Vertex> &SfMInitializer::getPoints3D() const { return points3D; }
const std::map<int, Eigen::Matrix4f> &SfMInitializer::getCameraPoses() const { return cameraPoses; }