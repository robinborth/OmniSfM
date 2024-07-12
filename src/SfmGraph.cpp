#include "SfmGraph.h"
#include <iostream>

SfMGraph::SfMGraph()
{}

int SfMGraph::addNode(const Node& node)
{
    this->cams.push_back(node);
    return this->cams.size() - 1;
}

void SfMGraph::addEdge(const Edge& edge)
{
    if (edge.node1_index >= this->cams.size() || edge.node2_index >= this->cams.size() || edge.node1_index == edge.node2_index) {
        std::cerr << "Invalid node indices in edge or self-loop detected." << std::endl;
        return;
    }
    this->edges.push_back(edge);
}

void SfMGraph::addPoint3D(const Point3D& point3D) 
{
    this->point3DList.push_back(point3D);
}

Node SfMGraph::getNode(int index) const
{
    for (const auto& cam : this->cams) 
    {
        if (cam.id == index) {
            return cam;
        }
    }
    std::cerr << "Node with index " << index << " not found." << std::endl;
    return Node();
}
