#pragma once


#include <string>
#include <vector>
#include "Eigen.h"

class PointCloud
{
public:
    PointCloud();

    PointCloud(float *depthMap, const Matrix3f &depthIntrinsics, const Matrix4f &depthExtrinsics, const unsigned width, const unsigned height, unsigned downsampleFactor = 1, float maxDistance = 0.1f);

    bool readFromFile(const std::string &filename);

    std::vector<Vector3f> &getPoints();

    const std::vector<Vector3f> &getPoints() const;

    std::vector<Vector3f> &getNormals();

    const std::vector<Vector3f> &getNormals() const;

    unsigned int getClosestPoint(Vector3f &p);

private:
    std::vector<Vector3f> m_points;
    std::vector<Vector3f> m_normals;
};
