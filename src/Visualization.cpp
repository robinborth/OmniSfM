#include "Visualization.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

Visualization::Visualization()
{
    _cameraMesh = SimpleMesh();
    _pointCloudMesh = SimpleMesh();
    _filename = "output";
}

Visualization::Visualization(const std::string &filename)
{
    _cameraMesh = SimpleMesh();
    _pointCloudMesh = SimpleMesh();
    _filename = filename;
}

std::vector<Eigen::Vector3f> Visualization::generateSyntheticPointCloud()
{
    std::vector<Eigen::Vector3f> points;

    // Define the vertices of a cube
    float side = 1.0f;
    std::vector<Eigen::Vector3f> cubeVertices;
    cubeVertices.emplace_back(-side, -side, -side);
    cubeVertices.emplace_back(side, -side, -side);
    cubeVertices.emplace_back(side, side, -side);
    cubeVertices.emplace_back(-side, side, -side);
    cubeVertices.emplace_back(-side, -side, side);
    cubeVertices.emplace_back(side, -side, side);
    cubeVertices.emplace_back(side, side, side);
    cubeVertices.emplace_back(-side, side, side);

    // Add the vertices to the point cloud
    for (const auto &vertex : cubeVertices)
    {
        points.push_back(vertex);
    }
    return points;
}

std::vector<Eigen::Matrix4f> Visualization::generateSyntheticCameraPoses()
{
    std::vector<Eigen::Matrix4f> cameraPoses;

    std::vector<int> ints = {1, 2, 3};
    // Define camera poses around the cube
    std::vector<Eigen::Vector3f> positions;
    positions.emplace_back(2.0f, 2.0f, 2.0f);
    positions.emplace_back(-2.0f, 2.0f, 2.0f);
    positions.emplace_back(2.0f, -2.0f, 2.0f);
    positions.emplace_back(-2.0f, -2.0f, 2.0f);
    positions.emplace_back(2.0f, 2.0f, -2.0f);
    positions.emplace_back(-2.0f, 2.0f, -2.0f);
    positions.emplace_back(2.0f, -2.0f, -2.0f);
    positions.emplace_back(-2.0f, -2.0f, -2.0f);

    for (const auto &pos : positions)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

        // Create a simple rotation matrix (looking at the origin)
        Eigen::Vector3f forward = -pos.normalized();
        Eigen::Vector3f right = Eigen::Vector3f(0, 1, 0).cross(forward).normalized();
        Eigen::Vector3f up = forward.cross(right);

        pose.block<3, 1>(0, 0) = right;
        pose.block<3, 1>(0, 1) = up;
        pose.block<3, 1>(0, 2) = forward;
        pose.block<3, 1>(0, 3) = pos;

        cameraPoses.push_back(pose);
    }
    return cameraPoses;
}

void Visualization::saveToPLY(const std::vector<Eigen::Vector3f> &points3D, const std::vector<Eigen::Matrix4f> &cameraPoses,
                              const std::string &filename)
{
    std::ofstream outFile(filename);
    // std::locale mylocale("de_DE");

    // outFile.imbue(mylocale);
    if (!outFile.is_open())
    {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
        return;
    }

    // Write PLY header
    outFile << "ply\n";
    outFile << "format ascii 1.0\n";
    outFile << "element vertex " << points3D.size() << "\n";
    outFile << "property float x\n";
    outFile << "property float y\n";
    outFile << "property float z\n";
    outFile << "property uchar red\n";
    outFile << "property uchar green\n";
    outFile << "property uchar blue\n";
    outFile << "element camera " << cameraPoses.size() << "\n";
    outFile << "property float view_px\n";
    outFile << "property float view_py\n";
    outFile << "property float view_pz\n";
    outFile << "property float x_axisx\n";
    outFile << "property float x_axisy\n";
    outFile << "property float x_axisz\n";
    outFile << "property float y_axisx\n";
    outFile << "property float y_axisy\n";
    outFile << "property float y_axisz\n";
    outFile << "property float z_axisx\n";
    outFile << "property float z_axisy\n";
    outFile << "property float z_axisz\n";
    outFile << "property float focal\n";
    outFile << "property float scalex\n";
    outFile << "property float scaley\n";
    outFile << "property float centerx\n";
    outFile << "property float centery\n";
    outFile << "end_header\n";

    // Write point cloud data
    for (const auto &point : points3D)
    {
        outFile << point.x() << " " << point.y() << " " << point.z() << " 255 255 255\n";
    }

    // Write camera positions
    for (const auto &pose : cameraPoses)
    {
        Eigen::Matrix3f rotation = pose.block<3, 3>(0, 0);
        Eigen::Vector3f translation = pose.block<3, 1>(0, 3);
        outFile << translation.x() << " " << translation.y() << " " << translation.z() << " ";
        outFile << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " ";
        outFile << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " ";
        outFile << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << " ";
        outFile << "0.0 1.0 1.0 0.5 0.5\n"; // Assuming some default camera parameters
    }

    outFile.close();
    std::cout << "Data saved to " << filename << std::endl;
}

void Visualization::debugCorrespondenceMatching()
{
    // Load the source and target mesh.
    const std::string filenameSource = std::string("EDIT!.off");
    const std::string filenameTarget = std::string("EDIT! reference .off");

    SimpleMesh sourceMesh;
    if (!sourceMesh.loadMesh(filenameSource))
    {
        std::cout << "Mesh file wasn't read successfully." << std::endl;
    }

    SimpleMesh targetMesh;
    if (!targetMesh.loadMesh(filenameTarget))
    {
        std::cout << "Mesh file wasn't read successfully." << std::endl;
    }

    // PointCloud source{ sourceMesh };
    // PointCloud target{ targetMesh };

    // // Search for matches using FLANN.
    // std::unique_ptr<NearestNeighborSearch> nearestNeighborSearch = std::make_unique<NearestNeighborSearchFlann>();
    // nearestNeighborSearch->setMatchingMaxDistance(0.0001f);
    // nearestNeighborSearch->buildIndex(target.getPoints());
    // auto matches = nearestNeighborSearch->queryMatches(source.getPoints());

    // // Visualize the correspondences with lines.
    // SimpleMesh resultingMesh = SimpleMesh::joinMeshes(sourceMesh, targetMesh, Matrix4f::Identity());
    // auto sourcePoints = source.getPoints();
    // auto targetPoints = target.getPoints();

    // for (unsigned i = 0; i < 100; ++i) { // sourcePoints.size()
    //     const auto match = matches[i];
    //     if (match.idx >= 0) {
    //         const auto& sourcePoint = sourcePoints[i];
    //         const auto& targetPoint = targetPoints[match.idx];
    //         resultingMesh = SimpleMesh::joinMeshes(SimpleMesh::cylinder(sourcePoint, targetPoint, 0.002f, 2, 15), resultingMesh, Matrix4f::Identity());
    //     }
    // }

    // resultingMesh.writeMesh(std::string("correspondences.off"));
}

int Visualization::writeCameraMesh()
{
    std::string cameraFilename = _filename + "_cameras.off";
    std::cout << cameraFilename << std::endl;
    if (!_cameraMesh.writeMesh(cameraFilename))
    {
        std::cerr << "Failed to write camera mesh! Check file path!" << std::endl;
        return -1;
    }
    return 1; // Supress warning of no return
}

void Visualization::addVertex(const cv::Point3f &point, const cv::Vec3b &color)
{
    Vertex point2Vertex = {{point.x, point.y, point.z, 1.0f}, {color[0], color[1], color[2], 255}};
    _pointCloudMesh.addVertex(point2Vertex);
}

void Visualization::addVertex(const std::vector<cv::Point3f> &points, const std::vector<cv::Vec3b> &colors)
{
    if (points.size() != colors.size())
    {
        std::cerr << "Error: The number of points and colors must match." << std::endl;
        return;
    }
    for (size_t i = 0; i < points.size(); ++i)
    {
        Vertex point2Vertex = {
            {points[i].x, points[i].y, points[i].z, 1.0f},
            {colors[i][0], colors[i][1], colors[i][2], 255}};
        this->_pointCloudMesh.addVertex(point2Vertex);
    }
}

void Visualization::addVertex(const std::vector<ColoredPoint3f> &points)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        auto p = points[i];
        Vertex point2Vertex = {
            {p.point.x, p.point.y, p.point.z, 1.0f},
            {p.color[0], p.color[1], p.color[2], 255}};
        this->_pointCloudMesh.addVertex(point2Vertex);
    }
}
void Visualization::addVertex(const std::vector<Eigen::Vector3f> &points, const std::vector<Vector4uc> &colors)
{
    if (points.size() != colors.size())
    {
        std::cerr << "Error: The number of points and colors must match." << std::endl;
        return;
    }
    for (size_t i = 0; i < points.size(); ++i)
    {
        Vertex point2Vertex = {
            {points[i][0], points[i][1], points[i][2], 1.0f},
            {colors[i][0], colors[i][1], colors[i][2], colors[i][3]}};
        this->_pointCloudMesh.addVertex(point2Vertex);
    }
}
// Function to generate color gradient
Vector4uc Visualization::generateColor(int count, int max_count)
{
    count++; // index starts at 0
    if (count < 1)
        count = 1;
    if (count > max_count)
        count = max_count;

    float ratio = static_cast<float>(count - 1) / (max_count - 1);

    unsigned char r = static_cast<unsigned char>((1.0f - ratio) * 255);
    unsigned char g = static_cast<unsigned char>(ratio * 255);
    unsigned char b = 0;

    return Vector4uc(r, g, b, 255);
}

void Visualization::addCamera(const Eigen::Matrix4f &cameraPose, const float scale, const Vector4uc &color)
{

    _cameraMesh = SimpleMesh::joinMeshes(SimpleMesh::camera(cameraPose, scale, color), _cameraMesh, Matrix4f::Identity());
}
void Visualization::addCamera(const Eigen::Matrix4f &cameraPose, const cv::Mat &intrinsic, const Vector4uc &color)
{
    double focalLength = intrinsic.at<double>(0, 0);
    // Determine the scale based on the focal length
    auto scale = static_cast<float>(focalLength * 0.00003);
    // std::cout << scale << std::endl;
    _cameraMesh = SimpleMesh::joinMeshes(SimpleMesh::camera(cameraPose, scale, color), _cameraMesh, Matrix4f::Identity());
}

void Visualization::addCamera(const std::vector<Eigen::Matrix4f> &cameraPoses, const float scale, const Vector4uc &color)
{
    for (size_t i = 0; i < cameraPoses.size(); i++)
    {
        addCamera(cameraPoses[i], scale, generateColor(i, cameraPoses.size()));
    }
}
void Visualization::addCamera(const std::vector<Eigen::Matrix4f> &cameraPoses, const std::vector<cv::Mat> &intrinsics)
{
    // set default scale if no intrinsic
    // use intrinsics focal length to determine scale of camera
    if (cameraPoses.size() != intrinsics.size())
    {
        std::cerr << "Error: The number of cameras and intrinsics must match." << std::endl;
        return;
    }
    for (size_t i = 0; i < cameraPoses.size(); i++)
    {
        addCamera(cameraPoses[i], intrinsics[i], generateColor(i, cameraPoses.size()));
    }
}

void Visualization::fillPointCloudWithDefaultValues()
{
    // SimpleMesh newPointCloudMesh;
    auto syntheticPoints = generateSyntheticPointCloud();
    for (const auto &point : syntheticPoints)
    {
        Vertex point2Vertex = {{point[0], point[1], point[2], 1.0f}, {255, 255, 255, 1}};
        _pointCloudMesh.addVertex(point2Vertex);
    }
}

void Visualization::fillCamerasWithDefaultValues()
{
    // SimpleMesh currentCameraMesh;
    auto synthetic_cameraPoses = generateSyntheticCameraPoses();
    for (const auto &pose : synthetic_cameraPoses)
    {
        addCamera(pose, 0.0005f);
    }
}

int Visualization::writePointCloudMesh()
{
    std::string pointCloudFilename = _filename + "_pointCloud.off";
    std::cout << pointCloudFilename << std::endl;
    if (!_pointCloudMesh.writeMesh(pointCloudFilename))
    {
        std::cerr << "Failed to write point cloud mesh! Check file path!" << std::endl;
        return -1;
    }
    return 1; // Suppress warning of no return
}

int Visualization::writeAllMeshes()
{
    if (writeCameraMesh() == -1 || writePointCloudMesh() == -1)
        return -1;
    return 1;
}