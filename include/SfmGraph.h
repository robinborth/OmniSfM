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
    void addEdge(const Edge& edge);
    void addPoint3D(const Point3D& point3D);
    Node getNode(int index) const;
    int getNumCams() const;
    int getNumEdges() const;
    int getNumPoint3D() const;
};