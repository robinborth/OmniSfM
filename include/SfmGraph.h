#pragma once

#include <vector>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include "Definitions.h"

class SfMGraph {
public:
    SfMGraph();
    std::vector<Node> cams;
    std::vector<Edge> edges;
    std::vector<Point3D> point3DList;

    int addNode(const Node& node);
    int getNumCams() const;
    int getNumEdges() const;
    int getNumPoint3D() const;
    bool isImageExist(int id) const;
    void addEdge(const Edge& edge);
    void addPoint3D(const Point3D& point3D);
    Node findCameraById(int id);
    Node getNode(int index) const;

    void updateAdjusted3DPointPoses(std::vector<Vertex> &points3D);
    void updateAdjustedIntrinsicParams(Eigen::Matrix<double, 4, 1> &intrinsics);
    void updateAdjustedExtrinsicParams(std::vector<Eigen::Matrix4f> &extrinsics);

    Eigen::Matrix<double, 6, 1> extractExtrinsics(const Eigen::Matrix4f &pose);
    Eigen::Matrix<double, 4, 1> extractIntrinsics();
    Eigen::Matrix<double, 3, 1> extractPoint3d(const Eigen::Vector4f &position);
    std::map<int, Eigen::Matrix<double, 6, 1>> extractAllExtrinsics();
    std::vector<Eigen::Matrix<double, 3, 1>> extractAllPoint3d();
    std::vector<Eigen::Matrix4f> constructPoseFromExtrinsics(const std::map<int, Eigen::Matrix<double, 6, 1>> &extrinsicsMap);
    std::vector<Vertex> construct3dPoints(const std::vector<Eigen::Matrix<double, 3, 1>> &point3ds);
};