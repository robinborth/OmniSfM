#include "SfMInitializer.h"

#include "ImageStorage.h"
#include <opencv2/flann.hpp>
#include <opencv2/core/eigen.hpp>
#include <limits>
#include "Eigen.h"

SfMInitializer::SfMInitializer(ImageStorage &imageStorage, SfMGraph &graph) : imageStorage(imageStorage), graph(graph) {}

cv::Mat SfMInitializer::getIntrinsic()
{
    cv::Mat k; // HACK convert to cv2
    cv::eigen2cv(this->imageStorage.getIntrinsics(), k);
    return k;
}

std::vector<cv::Point3f> SfMInitializer::convertVerticesToCvPoint3f(const std::vector<Vertex> &vertices)
{
    std::vector<cv::Point3f> objectPoints;
    objectPoints.reserve(vertices.size());
    for (const auto &vertex : vertices)
    {
        objectPoints.emplace_back(cv::Point3f(vertex.position(0), vertex.position(1), vertex.position(2)));
    }
    return objectPoints;
}

bool SfMInitializer::estimateInitialPose(const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2, cv::Mat &R, cv::Mat &t)
{
    cv::Mat K = getIntrinsic();
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0);
    if (E.empty())
    {
        std::cerr << "Essential matrix estimation failed." << std::endl;
        return false;
    }

    // Recover pose from the Essential matrix
    int inliers = cv::recoverPose(E, pts1, pts2, K, R, t);
    std::cout << "Recovered pose with " << inliers << " inliers." << std::endl;

    // adjust scale of translation from m in cm;
    t /= 100;
    return inliers > 0;
}

cv::Mat SfMInitializer::world2Image(Image img)
{
    cv::Mat K;
    cv::eigen2cv(img.K, K);

    Eigen::Matrix<float, 3, 4> p = img.P.topRows<3>();
    cv::Mat P(3, 4, CV_32F);

    // Copy data from Eigen matrix to cv::Mat
    cv::eigen2cv(p, P);

    return K * P;
}

std::vector<Vertex> SfMInitializer::triangulatePointsWithColor(const std::vector<cv::Point2f> &pts1,
                                                               const std::vector<cv::Point2f> &pts2,
                                                               const std::vector<cv::Vec3b> &colors1,
                                                               const std::vector<cv::Vec3b> &colors2,
                                                               const cv::Mat &P1,
                                                               const cv::Mat &P2)
{
    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);
    std::vector<Vertex> vertices;
    for (int i = 0; i < points4D.cols; ++i)
    {
        cv::Vec4d point = points4D.col(i);
        point /= point[3]; // Normalize to convert from homogeneous to Cartesian coordinates
        // HACK camera system definition
        // point[0] = -point[0];
        // point[1] = -point[1];
        // point[2] = -point[2];

        // Average the colors from both images
        Vector4uc color(
            (colors1[i][0] + colors2[i][0]) / 2,
            (colors1[i][1] + colors2[i][1]) / 2,
            (colors1[i][2] + colors2[i][2]) / 2,
            255 // Alpha channel set to maximum
        );
        Vertex vertex{{point[0], point[1], point[2], 1.0f}, color};
        this->points3D.push_back(vertex);
        vertices.push_back(vertex);
    }

    return vertices;
}

bool SfMInitializer::refineCameraPoseWithPnP(const std::vector<Vertex> &objectPoints,
                                             const std::vector<cv::Point2f> &imagePoints,
                                             size_t cameraId,
                                             const cv::Mat &R,
                                             const cv::Mat &t)
{
    // Prepare rotation and translation vectors for the solvePnP function
    cv::Mat rvec, tvec, k;
    cv::Rodrigues(R, rvec);
    cv::eigen2cv(this->imageStorage.getIntrinsics(), k);

    // Execute the solvePnP function to refine the pose
    std::vector<cv::Point3f> objectPointsCv = convertVerticesToCvPoint3f(objectPoints);
    bool success = cv::solvePnP(objectPointsCv, imagePoints, k, cv::noArray(), rvec, t, true, cv::SOLVEPNP_ITERATIVE);

    if (success)
    {
        cv::Rodrigues(rvec, R); // Convert rotation vector back to rotation matrix
        std::cout << "Pose refinement successful for camera " << cameraId << std::endl;

        Eigen::Matrix4f refinedPose = combineRotationAndTranslationIntoMatrix(R, t);
        this->imageStorage.updatePose(cameraId, refinedPose);
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

std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f>, std::vector<cv::Vec3b>, std::vector<cv::Vec3b>> SfMInitializer::extractMatchedPoints(const std::vector<cv::DMatch> &matches, size_t idx1, size_t idx2)
{
    std::vector<cv::Point2f> pts1, pts2;
    std::vector<cv::Vec3b> colors1, colors2;
    std::cout << "Extracting Matched Points for id " << idx1 << " and id " << idx2 << std::endl;
    Image *img1 = imageStorage.findImage(idx1);
    Image *img2 = imageStorage.findImage(idx2);
    if (!img1 || !img2)
    {
        std::cerr << "Image not found." << std::endl;
        return {pts1, pts2, colors1, colors2};
    }
    for (const auto &match : matches)
    {
        cv::Point2f pt1 = img1->keypoints[match.queryIdx].pt;
        cv::Point2f pt2 = img2->keypoints[match.trainIdx].pt;
        pts1.push_back(pt1);
        pts2.push_back(pt2);
        colors1.push_back(img1->rgb.at<cv::Vec3b>((int)pt1.y, (int)pt1.x));
        colors2.push_back(img2->rgb.at<cv::Vec3b>((int)pt2.y, (int)pt2.x));
    }
    return {pts1, pts2, colors1, colors2};
}

std::vector<Eigen::Vector4f> SfMInitializer::vertex2Camera(size_t imgIdx, std::vector<Vertex> points3D)
{
    std::vector<Eigen::Vector4f> points;
    auto img = imageStorage.findImage(imgIdx);
    if (!img)
    {
        std::cerr << "Image not found." << std::endl;
        return points;
    }
    for (auto i = 0; i < points3D.size(); ++i)
    {
        auto p = img->P * points3D[i].position;
        points.push_back(p);
    }
    return points;
}

void SfMInitializer::solveDepthMaps(size_t imgIdx, std::vector<cv::Point2f> points2D, std::vector<Eigen::Vector4f> points3D)
{
    // compute the z statistics from the gt points
    Eigen::VectorXf p(points3D.size());
    for (size_t i = 0; i < points3D.size(); ++i)
    {
        p(i) = points3D[i][2];
    }
    float mean = p.mean();
    float variance = (p.array() - mean).square().sum() / (p.size() - 1);
    float stddev = std::sqrt(variance);
    std::cout << mean << " " << variance << " " << stddev << std::endl;

    // Optimze for the scale and translation in the depth maps
    auto img = imageStorage.findImage(imgIdx);
    if (!img)
    {
        std::cerr << "Image not found." << std::endl;
        return;
    }
    std::vector<float> gtDepths, imgDepths;
    for (auto i = 0; i < points3D.size(); ++i)
    {
        auto imgDepth = img->depth((int)points2D[i].y, (int)points2D[i].x);
        auto gtDepth = points3D[i][2];

        float gt_std = std::abs(gtDepth - mean) / stddev;
        if (gt_std > 1.0) // if the z-value is further away then 1 std this could be an outlier
        {
            std::cout << "Skip: " << gtDepth << " " << imgDepth << std::endl;
            continue;
        }

        imgDepths.push_back(imgDepth);
        gtDepths.push_back(gtDepth); // z-value
        if (i < 5)                   // debugging
            std::cout << gtDepth << " " << imgDepth << " " << points3D[i] << std::endl;
    }

    // solve for scale and
    const unsigned nPoints = imgDepths.size();
    MatrixXf A(nPoints, 2);
    VectorXf b(nPoints);
    for (int i = 0; i < nPoints; ++i)
    {
        A(i, 0) = imgDepths[i];
        A(i, 1) = 1.0;
        b(i) = gtDepths[i];
    }
    Matrix2f ATA = A.transpose() * A;
    Vector2f ATb = A.transpose() * b;
    JacobiSVD<Matrix2f> svd(ATA, ComputeFullU | ComputeFullV);
    VectorXf solution = svd.solve(ATb);
    float w = solution[0]; // scale
    float q = solution[1]; // shift
    imageStorage.updateScale(imgIdx, w);
    imageStorage.updateShift(imgIdx, q);
    std::cout << "w: " << w << " q: " << q << std::endl;
}

void SfMInitializer::runSfM(ImagePairMatches &allMatches)
{
    if (allMatches.empty())
    {
        std::cout << "No matches to process." << std::endl;
        return;
    }

    // First 2 images estimate the Pose between them!
    const auto &pair = allMatches.begin()->first;
    const auto &matches = allMatches.begin()->second;
    imageStorage.updatePose(pair.first, Eigen::Matrix4f::Identity()); // world2camera
    std::cout << pair.first << " " << pair.second << " " << matches.size() << std::endl;
    if (matches.size() < 60)
    {
        std::cout << "Not enough matches for image pair (" << pair.first << ", " << pair.second << ")." << std::endl;
        return;
    }
    auto [pts1, pts2, colors1, colors2] = extractMatchedPoints(matches, pair.first, pair.second);

    std::vector<Vertex> points3D;
    cv::Mat R, t;
    if (estimateInitialPose(pts1, pts2, R, t)) // from pts1 -> pts2 (note that pts1 is world for first frame)
    {
        imageStorage.updatePose(pair.second, combineRotationAndTranslationIntoMatrix(R, t));
        Image *img1 = imageStorage.findImage(pair.first);
        Image *img2 = imageStorage.findImage(pair.second);
        if (!img1 || !img2)
        {
            std::cerr << "Image not found." << std::endl;
            return;
        }
        auto P1 = world2Image(*img1);
        auto P2 = world2Image(*img2);
        points3D = triangulatePointsWithColor(pts1, pts2, colors1, colors2, P1, P2);
    }
    else
    {
        std::cout << "Failed to estimate pose for image pair (" << pair.first << ", " << pair.second << ")." << std::endl;
    }

    std::cout << "Solve depth map for image: " << pair.first << std::endl;
    auto c1 = vertex2Camera(pair.first, points3D);
    solveDepthMaps(pair.first, pts1, c1);
    std::cout << "Solve depth map for image: " << pair.second << std::endl;
    auto c2 = vertex2Camera(pair.second, points3D);
    solveDepthMaps(pair.second, pts2, c2);
}

void SfMInitializer::updateGraph(const std::vector<Vertex> &points3D, const std::vector<cv::DMatch> &matches, Image *img1, Image *img2)
{
    Node node1 = {(int)img1->id, img1->P, img1->keypoints, img1->K};
    Node node2 = {(int)img2->id, img2->P, img2->keypoints, img2->K};
    int nodeIndex1 = graph.addNode(node1);
    int nodeIndex2 = graph.addNode(node2);

    for (size_t i = 0; i < points3D.size(); ++i)
    {
        Point3D p3d;
        p3d.position = Eigen::Vector4f(points3D[i].position[0], points3D[i].position[1], points3D[i].position[2], 1.0f);
        p3d.observations.push_back(std::make_pair(img1->id, matches[i].queryIdx)); // source image and keypoint
        p3d.observations.push_back(std::make_pair(img2->id, matches[i].trainIdx)); // target image and keypoint
        p3d.color = points3D[i].color;
        graph.addPoint3D(p3d);
    }
    // Add edge to the graph representing the matches between these two images
    Edge edge = {nodeIndex1, nodeIndex2, matches};
    graph.addEdge(edge);
}

void SfMInitializer::twoViewSfm(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2)
{
    if (imgId1 >= this->imageStorage.getNumImages() || imgId2 >= this->imageStorage.getNumImages() || imgId1 == imgId2) {
        std::cerr << "Invalid image indices provided. Indices must be within the range of the image vector and not equal." << std::endl;
    }
    Image *sourceImg = this->imageStorage.findImage(imgId1);
    Image *targetImg = this->imageStorage.findImage(imgId2);
    if (!sourceImg || !targetImg) {
        std::cerr << "Error: Could not find images." << std::endl;
        return;
    }

    // Check if there are enough matches to proceed
    if (matchesForPair.size() < 60) {
        std::cout << "Not enough matches to estimate a reliable pose (" << matchesForPair.size() << " matches found)." << std::endl;
    }

    // Extract points and colors for the matched keypoints
    auto [pts1, pts2, colors1, colors2] = extractMatchedPoints(matchesForPair, imgId1, imgId2);
    imageStorage.updatePose(imgId1, Eigen::Matrix4f::Identity()); // world2camera
    poses.push_back(Eigen::Matrix4f::Identity());

    std::cout << "==> Extracted " << pts1.size() << " points for pose estimation." << std::endl;
    std::cout << "==> Extracted " << pts2.size() << " points for pose estimation." << std::endl;

    std::vector<Vertex> points3D;
    cv::Mat R, t;
    if (estimateInitialPose(pts1, pts2, R, t)) // from pts1 -> pts2 (note that pts1 is world for first frame)
    {
        imageStorage.updatePose(imgId2, combineRotationAndTranslationIntoMatrix(R, t));
        poses.push_back(combineRotationAndTranslationIntoMatrix(R, t));
        auto P1 = world2Image(*sourceImg);
        auto P2 = world2Image(*targetImg);
        points3D = triangulatePointsWithColor(pts1, pts2, colors1, colors2, P1, P2);
        updateGraph(points3D, matchesForPair, sourceImg, targetImg);
    }
    else
    {
        std::cout << "Failed to estimate pose for image pair (" << imgId1 << ", " << imgId2 << ")." << std::endl;
    }

    std::cout << "Solve depth map for image: " << imgId1 << std::endl;
    auto c1 = vertex2Camera(imgId1, points3D);
    solveDepthMaps(imgId1, pts1, c1);
    std::cout << "Solve depth map for image: " << imgId2 << std::endl;
    auto c2 = vertex2Camera(imgId2, points3D);
    solveDepthMaps(imgId2, pts2, c2);
}


const std::vector<Vertex> &SfMInitializer::getPoints3D() const { return points3D; }
const std::vector<Eigen::Matrix4f> SfMInitializer::getCameraPoses() const
{
    return poses;
}