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

bool SfMInitializer::refineCameraPoseWithPnP(const std::vector<cv::Point3f> &objectPointsCv,
                                             const std::vector<cv::Point2f> &imagePoints,
                                             size_t cameraId,
                                             cv::Mat &R,
                                             cv::Mat &t)
{
    // Prepare rotation and translation vectors for the solvePnP function
    cv::Mat rvec, k;
    cv::eigen2cv(this->imageStorage.getIntrinsics(), k);

    // Execute the solvePnP function to refine the pose
    //std::vector<cv::Point3f> objectPointsCv = convertVerticesToCvPoint3f(objectPoints);
    bool success = cv::solvePnP(objectPointsCv, imagePoints, k, cv::noArray(), rvec, t, false, cv::SOLVEPNP_ITERATIVE);
    if (success)
    {
        cv::Rodrigues(rvec, R); // Convert rotation vector back to rotation matrix
        std::cout << "Pose refinement successful for camera " << cameraId << std::endl;
        return success;
    }

    std::cout << "Pose refinement FAILED for camera " << cameraId << std::endl;
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
    //std::cout << "Extracting Matched Points for id " << idx1 << " and id " << idx2 << std::endl;
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


void SfMInitializer::addSfM(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2)
{
    Image *sourceImg = this->imageStorage.findImage(imgId1);
    Image *targetImg = this->imageStorage.findImage(imgId2);
    if (!sourceImg || !targetImg)
    {
        std::cerr << "Error: Could not find images." << std::endl;
        return;
    }

    // imgId1 is assumed to be already in the graph structure
    if (!graph.isImageExist(imgId1))
    {
        std::cout << "ERROR: image is not integrated in graph id: " << imgId1 << std::endl;
    }

    // Extract 3D points from graph of img1 that are shared with img2
    std::vector<Vertex> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    for (auto &point3D : graph.point3DList)
    {
        for (auto &observation : point3D.observations)
        {
            // check if the point3D is from the image1 that is our reference to register image2
            if (observation.first == imgId1) // the imageId of image1
            {
                for (const auto &match : matchesForPair) // go trough all matches and only add the ones that are registerd
                {
                    auto observationKeypointIdImg1 = observation.second; // the keypointId of image1
                    auto keypointIdImg1 = match.queryIdx;                // the keypointId of image1
                    auto keypointIdImg2 = match.trainIdx;
                    if (observationKeypointIdImg1 == keypointIdImg1) // the keypoints for image1 are registerd we have a match for image2
                    {
                        // the keypointId of image2
                        cv::Point2f pt2 = targetImg->keypoints[keypointIdImg2].pt; // get the 2d point from the image to add
                        imagePoints.push_back(pt2);
                        auto vertex = Vertex{point3D.position, point3D.color};
                        objectPoints.push_back(vertex);
                    }
                }
            }
        }
    }

    // extract new camera pose for image2
    cv::Mat R, t; // this needs to be estimated
    if (refineCameraPoseWithPnP(objectPoints, imagePoints, imgId2, R, t))
    {
        std::cout << "PnP success for id: " << imgId2 << std::endl;
        imageStorage.updatePose(imgId2, combineRotationAndTranslationIntoMatrix(R, t));
        poses.push_back(combineRotationAndTranslationIntoMatrix(R, t));
    }
    else
    {
        std::cout << "ERROR: PnP failed for id: " << imgId2 << std::endl;
    }

    // after estimation of the pose for the added image we need to insert all the points
    // which has a match between image1 and image2 this can be more then the one that are
    // in the shared set (e.g. we need this to add more points to the current pointcloud)
    std::vector<Vertex> fullPoints3D; // hold keypoints that are not in the shared set
    auto P1 = world2Image(*sourceImg);
    auto P2 = world2Image(*targetImg);
    auto [pts1, pts2, colors1, colors2] = extractMatchedPoints(matchesForPair, imgId1, imgId2);
    fullPoints3D = triangulatePointsWithColor(pts1, pts2, colors1, colors2, P1, P2);
    // note that in update graph we call addPoint3D, which checks if the point allready is
    // in the shared point cloud, if so we just add the observation (keypoint) for the second
    // image, and don't add a new point, if the point however is new we add the point to the
    // pointcloud.
    updateGraph(fullPoints3D, matchesForPair, sourceImg, targetImg);
}

void SfMInitializer::updateGraph(const std::vector<Vertex> &points3D, const std::vector<cv::DMatch> &matches, Image *img1, Image *img2)
{
    Node node1 = {(int)img1->id, img1->P, img1->keypoints, img1->K};
    Node node2 = {(int)img2->id, img2->P, img2->keypoints, img2->K};
    int nodeIndex1 = graph.addNode(node1);
    int nodeIndex2 = graph.addNode(node2);
    //std::cout << "Number of 3DPoints before update " << graph.getNumPoint3D() << std::endl;

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
    //std::cout << "Number of 3DPoints after update " << graph.getNumPoint3D() << std::endl;
}

void SfMInitializer::twoViewSfm(const std::vector<cv::DMatch> &matchesForPair, size_t imgId1, size_t imgId2)
{
    Image *sourceImg = this->imageStorage.findImage(imgId1);
    Image *targetImg = this->imageStorage.findImage(imgId2);
    if (!sourceImg || !targetImg)
    {
        std::cerr << "Error: Could not find images." << std::endl;
        return;
    }

    // Check if there are enough matches to proceed
    if (matchesForPair.size() < 60)
    {
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

    // TODO add again
    // std::cout << "Solve depth map for image: " << imgId1 << std::endl;
    // auto c1 = vertex2Camera(imgId1, points3D);
    // solveDepthMaps(imgId1, pts1, c1);
    // std::cout << "Solve depth map for image: " << imgId2 << std::endl;
    // auto c2 = vertex2Camera(imgId2, points3D);
    // solveDepthMaps(imgId2, pts2, c2);
}


void SfMInitializer::multiViewSfm(const ImagePairMatches &allMatches, int newImageId)
{
    Image *newImg = this->imageStorage.findImage(newImageId);
    if (!newImg)
    {
        std::cerr << "Error: Could not find image." << std::endl;
        return;
    }

    std::vector<int> imgIds = this->graph.getCameraIds();
    ImagePairMatches matchesForNewImg;
    std::vector<cv::Point3f> objectPoints;  // To store 3D coordinates
    std::vector<cv::Point2f> imagePoints; 

    Image *sourceImg = NULL;

    for (const auto &imgId : imgIds)
    {
        auto matches = allMatches.at(std::make_pair(imgId, newImageId));
        // store those matches
        matchesForNewImg[std::make_pair(imgId, newImageId)] = matches;
    }

    std::vector<std::vector<Vertex>> fullPoints3D; // hold keypoints that are not in the shared set
    for (auto &matchList : matchesForNewImg)
    {
        sourceImg = this->imageStorage.findImage(matchList.first.first);
        auto P1 = world2Image(*sourceImg);
        auto P2 = world2Image(*newImg);
        auto [pts1, pts2, colors1, colors2] = extractMatchedPoints(matchList.second, sourceImg->id, newImg->id);
        std::vector<Vertex> temp3d;
        temp3d = triangulatePointsWithColor(pts1, pts2, colors1, colors2, P1, P2);
        fullPoints3D.push_back(temp3d);
        //std::cout << "Number of 3DPoints after update " << temp3d.size() << std::endl;
        for (auto &match : matchList.second)
        {
            for (const auto &point3d : this->graph.point3DList)
            {
                auto it = std::find_if(point3d.observations.begin(), point3d.observations.end(), [&](const std::pair<int, int> &obs) {
                    return obs.first == matchList.first.first && obs.second == match.queryIdx;
                });
                if (it != point3d.observations.end()) 
                {
                    // If found, add the 3D point and the corresponding 2D point in Image 2 to the lists
                    objectPoints.push_back(cv::Point3f(point3d.position.x(), point3d.position.y(), point3d.position.z()));
                    imagePoints.push_back(newImg->keypoints[match.trainIdx].pt);
                }
            }
        }
        
    }

    //std::cout << "==> Extracted " << objectPoints.size() << " points for pose estimation." << std::endl;

    cv::Mat R, t; // this needs to be estimated
    if (refineCameraPoseWithPnP(objectPoints, imagePoints, newImg->id, R, t))
    {
        std::cout << "PnP success for id: " << newImg->id << std::endl;
        imageStorage.updatePose(newImg->id, combineRotationAndTranslationIntoMatrix(R, t));
        poses.push_back(combineRotationAndTranslationIntoMatrix(R, t));
    }
    else
    {
        std::cout << "ERROR: PnP failed for id: " << newImg->id << std::endl;
    }

    for (size_t i = 0; i < fullPoints3D.size(); ++i)
    {
        //std::cout << "Number of 3DPoints before update " << graph.getNumPoint3D() << std::endl;
        updateGraph(fullPoints3D[i], matchesForNewImg.at(std::make_pair(imgIds[i], newImageId)), sourceImg, newImg);
        //std::cout << "Number of 3DPoints after update " << graph.getNumPoint3D() << std::endl;
    }
}

const std::vector<Vertex> &SfMInitializer::getPoints3D() const { return points3D; }
const std::vector<Eigen::Matrix4f> SfMInitializer::getCameraPoses() const
{
    return poses;
}