#include "SfmGraph.h"
#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

SfMGraph::SfMGraph()
{
}

int SfMGraph::addNode(const Node &node)
{
    bool nodeExist = false; // ensure that we don't insert duplicates
    for (auto &cam : this->cams)
    {
        if (cam.id == node.id)
        {
            nodeExist = true;
        }
    }
    if (!nodeExist)
    {
        this->cams.push_back(node);
    }
    return this->cams.size() - 1;
}

void SfMGraph::addEdge(const Edge &edge)
{
    this->edges.push_back(edge);
}

void SfMGraph::addPoint3D(const Point3D &point3D)
{
    bool pointExist = false; // ensure that we don't insert duplicates
    for (auto &sharedPoint3D : point3DList)
    {
        for (auto &sharedObservation : sharedPoint3D.observations)
        {
            // check if the 3D point is already in the shared point3DList
            for (auto &observation : point3D.observations)
            {
                if (observation.first == sharedObservation.first & observation.second == sharedObservation.second)
                {
                    pointExist = true;
                }
            }
        }
        // just insert the observations into the shared points
        if (pointExist)
        {
            for (auto &observation : point3D.observations)
            {
                sharedPoint3D.observations.push_back(observation);
            }
            break; // skip the serach
        }
    }
    // the 3D point is a new one, hence add it to the point3DList
    if (!pointExist)
    {
        this->point3DList.push_back(point3D);
    }
}

Node SfMGraph::getNode(int index) const
{
    for (const auto &cam : this->cams)
    {
        if (cam.id == index)
        {
            return cam;
        }
    }
    std::cerr << "Node with index " << index << " not found." << std::endl;
    return Node();
}

int SfMGraph::getNumCams() const
{
    return this->cams.size();
}

int SfMGraph::getNumEdges() const
{
    return this->edges.size();
}

int SfMGraph::getNumPoint3D() const
{
    return this->point3DList.size();
}

void SfMGraph::updateAdjusted3DPointPoses(std::vector<Vertex> &points3D)
{
    for (size_t i = 0; i < this->point3DList.size(); i++)
    {
        Point3D point = this->point3DList[i];
        point.position = Eigen::Vector4f(points3D[i].position(0), points3D[i].position(1), points3D[i].position(2), 1.0);
    }
}

void SfMGraph::updateAdjustedIntrinsicParams(Eigen::Matrix<double, 4, 1> &intrinsics)
{
    for (size_t i = 0; i < this->cams.size(); i++)
    {
        this->cams[i].intrinsics(0, 0) = intrinsics(0);
        this->cams[i].intrinsics(1, 1) = intrinsics(1);
        this->cams[i].intrinsics(0, 2) = intrinsics(2);
        this->cams[i].intrinsics(1, 2) = intrinsics(3);
    }
}

void SfMGraph::updateAdjustedExtrinsicParams(std::vector<Eigen::Matrix4f> &extrinsics)
{
    for (size_t i = 0; i < this->cams.size(); i++)
    {
        this->cams[i].pose = extrinsics[i];
    }
}

Node SfMGraph::findCameraById(int id)
{
    for (const auto &cam : this->cams)
    {
        if (cam.id == id)
        {
            return cam;
        }
    }
    std::cerr << "Camera with id " << id << " not found." << std::endl;
    return Node();
}

std::map<int, Eigen::Matrix<double, 6, 1>> SfMGraph::extractAllExtrinsics()
{
    std::map<int, Eigen::Matrix<double, 6, 1>> extrinsics;
    for (const auto &cam : this->cams)
    {
        extrinsics[cam.id] = extractExtrinsics(cam.pose);
    }
    std::cout << "Extracted all extrinsics" << extrinsics.size() << std::endl;
    return extrinsics;
}

Eigen::Matrix<double, 6, 1> SfMGraph::extractExtrinsics(const Eigen::Matrix4f &pose)
{
    // Initialize the resulting extrinsics array
    Eigen::Matrix<double, 6, 1> extrinsicsArr;

    // Extract and convert the rotation matrix
    Eigen::Matrix3d rotationMatrix = pose.block<3, 3>(0, 0).cast<double>();

    // Convert rotation matrix to angle-axis
    ceres::RotationMatrixToAngleAxis(&rotationMatrix(0, 0), &extrinsicsArr(0, 0));

    // Directly place the translation elements
    extrinsicsArr(3, 0) = static_cast<double>(pose(0, 3));
    extrinsicsArr(4, 0) = static_cast<double>(pose(1, 3));
    extrinsicsArr(5, 0) = static_cast<double>(pose(2, 3));

    return extrinsicsArr;
}

Eigen::Matrix<double, 4, 1> SfMGraph::extractIntrinsics()
{
    Eigen::Matrix3f &matrix = this->cams[1].intrinsics;
    Eigen::Matrix<double, 4, 1> intrinsics; // for fx, fy, cx, cy
    // cast matrix to double
    Eigen::Matrix<double, 3, 3> matrix_double = matrix.cast<double>();

    intrinsics(0) = matrix_double(0, 0); // fx
    intrinsics(1) = matrix_double(1, 1); // fy
    intrinsics(2) = matrix_double(0, 2); // cx
    intrinsics(3) = matrix_double(1, 2); // cy

    return intrinsics;
}

Eigen::Matrix<double, 3, 1> SfMGraph::extractPoint3d(const Eigen::Vector4f &position)
{
    Eigen::Matrix<double, 3, 1> pointArr;
    for (int i = 0; i < 3; i++)
    {
        pointArr(i) = position(i);
    }
    return pointArr;
}

std::vector<Eigen::Matrix<double, 3, 1>> SfMGraph::extractAllPoint3d()
{
    std::vector<Eigen::Matrix<double, 3, 1>> point3dArr;
    for (const auto &point3d : this->point3DList)
    {
        point3dArr.push_back(extractPoint3d(point3d.position));
    }
    return point3dArr;
}

std::vector<Eigen::Matrix4f> SfMGraph::constructPoseFromExtrinsics(const std::map<int, Eigen::Matrix<double, 6, 1>> &extrinsicsMap)
{
    std::vector<Eigen::Matrix4f> poses;
    for (const auto &extrinsic : extrinsicsMap)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

        // Extract the rotation vector and translation vector from the current extrinsics element
        Eigen::Matrix<double, 3, 1> rotation_vec = extrinsic.second.head<3>();
        Eigen::Matrix<double, 3, 1> translation_vec = extrinsic.second.tail<3>();

        Eigen::Matrix3d rotationMatrix;
        ceres::AngleAxisToRotationMatrix(rotation_vec.data(), rotationMatrix.data());

        // Cast the rotation matrix from double to float and insert it into the pose matrix
        pose.block<3, 3>(0, 0) = rotationMatrix.cast<float>();

        // Cast the translation vector from double to float and insert it into the pose matrix
        pose.block<3, 1>(0, 3) = translation_vec.cast<float>();

        poses.push_back(pose);
    }

    return poses;
}

std::vector<Vertex> SfMGraph::construct3dPoints(const std::vector<Eigen::Matrix<double, 3, 1>> &point3ds)
{
    std::vector<Vertex> points3D;
    for (size_t i = 0; i < point3ds.size(); i++)
    {
        Eigen::Matrix<double, 3, 1> point3d = point3ds[i];
        Point3D point = this->point3DList[i];
        Vertex vertex;
        vertex.position = Eigen::Vector4f(point3d(0), point3d(1), point3d(2), 1.0);
        vertex.color = point.color;
        points3D.push_back(vertex);
    }
    return points3D;
}

bool SfMGraph::isImageExist(int id) const
{
    for (const auto &cam : this->cams)
    {
        if (cam.id == id)
        {
            return true;
        }
    }
    return false;
}