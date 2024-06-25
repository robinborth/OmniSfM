#pragma once

#include "Eigen.h"
#include "SimpleMesh.h"

class Visualization
{
public:
    Visualization();
    Visualization(const std::string filename);

    void addVertex(const cv::Point3f &point);
    void addVertex(const std::vector<cv::Point3f> &points, const std::vector<cv::Vec3b> &colors);
    void addVertex(const std::vector<Eigen::Vector3f> &points, const std::vector<Vector4uc> &colors);
    void addCamera(const Eigen::Matrix4f &cameraPose);
    void addCamera(const std::vector<Eigen::Matrix4f> &cameraPoses);
    void fillPointCloudWithDefaultValues();
    void fillCamerasWithDefaultValues();

    int writeCameraMesh();
    int writePointCloudMesh();
    int writeAllMeshes();

private:
    static void debugCorrespondenceMatching();
    static std::vector<Eigen::Vector3f> generateSyntheticPointCloud(); // Function to create a simple point cloud of a cube

    static std::vector<Eigen::Matrix4f> generateSyntheticCameraPoses(); // Function to create camera poses around the cube
    static void saveToPLY(const std::vector<Eigen::Vector3f> &points3D, const std::vector<Eigen::Matrix4f> &cameraPoses,
                          const std::string &filename);

    SimpleMesh _cameraMesh;
    SimpleMesh _pointCloudMesh;
    std::string _filename;
};