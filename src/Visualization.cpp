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

Visualization::Visualization(const std::string filename)
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
    cubeVertices.push_back({-side, -side, -side});
    cubeVertices.push_back({side, -side, -side});
    cubeVertices.push_back({side, side, -side});
    cubeVertices.push_back({-side, side, -side});
    cubeVertices.push_back({-side, -side, side});
    cubeVertices.push_back({side, -side, side});
    cubeVertices.push_back({side, side, side});
    cubeVertices.push_back({-side, side, side});
    /*= {
            {-side, -side, -side}, {side, -side, -side}, {side, side, -side}, {-side, side, -side},
            {-side, -side, side}, {side, -side, side}, {side, side, side}, {-side, side, side}
    };*/

    // Add the vertices to the point cloud
    for (const auto &vertex: cubeVertices) {
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
    positions.push_back({2.0f, 2.0f, 2.0f});
    positions.push_back({-2.0f, 2.0f, 2.0f});
    positions.push_back({2.0f, -2.0f, 2.0f});
    positions.push_back({-2.0f, -2.0f, 2.0f});
    positions.push_back({2.0f, 2.0f, -2.0f});
    positions.push_back({-2.0f, 2.0f, -2.0f});
    positions.push_back({2.0f, -2.0f, -2.0f});
    positions.push_back({-2.0f, -2.0f, -2.0f});

    /*{
    {2.0f, 2.0f, 2.0f}, {-2.0f, 2.0f, 2.0f}, {2.0f, -2.0f, 2.0f}, {-2.0f, -2.0f, 2.0f},
    {2.0f, 2.0f, -2.0f}, {-2.0f, 2.0f, -2.0f}, {2.0f, -2.0f, -2.0f}, {-2.0f, -2.0f, -2.0f}
};
*/

    for (const auto &pos: positions) {
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
    //std::locale mylocale("de_DE");

    //outFile.imbue(mylocale);
    if (!outFile.is_open()) {
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
    for (const auto &point: points3D) {
        outFile << point.x() << " " << point.y() << " " << point.z() << " 255 255 255\n";
    }

    // Write camera positions
    for (const auto &pose: cameraPoses) {
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
    if (!sourceMesh.loadMesh(filenameSource)) {
        std::cout << "Mesh file wasn't read successfully." << std::endl;
    }

    SimpleMesh targetMesh;
    if (!targetMesh.loadMesh(filenameTarget)) {
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
    std::stringstream ss_camera;
    ss_camera << _filename << "_cameras" << ".off";
    std::cout << _filename << "_cameras" << ".off" << std::endl;
    if (!this->_cameraMesh.writeMesh(ss_camera.str())) {
        std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
        return -1;
    }
    return 1; // Supress warning of no return
}

void Visualization::addVertex(const cv::Point3f &point)
{
    Vertex point2Vertex = {{point.x,point.y,point.z,1.0f},{255,255,255,1}};
    this->_pointCloudMesh.addVertex(point2Vertex);
}

void Visualization::addVertex(const std::vector<cv::Point3f> &points, const std::vector<cv::Vec3b> &colors) 
{
    if (points.size() != colors.size()) {
        std::cerr << "Error: The number of points and colors must match." << std::endl;
        return;
    }
    for (size_t i = 0; i < points.size(); ++i) {
        Vertex point2Vertex = {
            {points[i].x, points[i].y, points[i].z, 1.0f},
            {colors[i][0], colors[i][1], colors[i][2], 255}
        };
        this->_pointCloudMesh.addVertex(point2Vertex);
    }
}

void Visualization::addCamera(const Eigen::Matrix4f &cameraPose)
{
    this->_cameraMesh =  SimpleMesh::joinMeshes(SimpleMesh::camera(cameraPose, 0.0005f), _cameraMesh, Matrix4f::Identity());
}

void Visualization::addCamera(const std::vector<Eigen::Matrix4f> &cameraPoses)
{
    for (const auto& pose : cameraPoses) {
        this->_cameraMesh =  SimpleMesh::joinMeshes(SimpleMesh::camera(pose, 0.5f), _cameraMesh, Matrix4f::Identity());
    }
}

void Visualization::fillPointCloudWithDefaultValues()
{
    //SimpleMesh newPointCloudMesh;
    auto syntheticPoints = generateSyntheticPointCloud();
    for (const auto& point : syntheticPoints) {
        Vertex point2Vertex = {{point[0],point[1],point[2],1.0f},{255,255,255,1}};
        _pointCloudMesh.addVertex(point2Vertex);
    }
}
void Visualization::fillCamerasWithDefaultValues()
{
    //SimpleMesh currentCameraMesh;
    auto synthetic_cameraPoses = generateSyntheticCameraPoses();
    for (const auto& pose : synthetic_cameraPoses) {
        _cameraMesh = SimpleMesh::joinMeshes(SimpleMesh::camera(pose, 0.0005f), _cameraMesh, Matrix4f::Identity());
    }
}

int Visualization::writePointCloudMesh()
{
    std::stringstream ss;
    ss << _filename << "_pointCloud" << ".off";
    std::cout << _filename << "_pointCloud" << ".off" << std::endl;
    if (!(this->_pointCloudMesh.writeMesh(ss.str()))) {
        std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
        return -1;
    }
    return 1; // Supress warning of no return
}

int Visualization::writeAllMeshes()
{
    if (writeCameraMesh() == -1 || writePointCloudMesh() == -1)
        return -1;
    return 1;
}