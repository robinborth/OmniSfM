#include "SfmGraph.h"
#include <iostream>

SfMGraph::SfMGraph()
{
}

int SfMGraph::addNode(const Node &node)
{
    this->cams.push_back(node);
    return this->cams.size() - 1;
}

void SfMGraph::addEdge(const Edge &edge)
{
    if (edge.node1_index >= this->cams.size() || edge.node2_index >= this->cams.size() || edge.node1_index == edge.node2_index)
    {
        std::cerr << "Invalid node indices in edge or self-loop detected." << std::endl;
        return;
    }
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
