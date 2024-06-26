#include "PointCloud.h"
#include <iostream>
#include <fstream>

PointCloud::PointCloud() {}

PointCloud::PointCloud(float *depthMap, const Matrix3f &depthIntrinsics, const Matrix4f &depthExtrinsics, const unsigned width, const unsigned height, unsigned downsampleFactor, float maxDistance)
{
    // Get depth intrinsics.
    float fovX = depthIntrinsics(0, 0);
    float fovY = depthIntrinsics(1, 1);
    float cX = depthIntrinsics(0, 2);
    float cY = depthIntrinsics(1, 2);
    const float maxDistanceHalved = maxDistance / 2.f;

    // Compute inverse depth extrinsics.
    Matrix4f depthExtrinsicsInv = depthExtrinsics.inverse();
    Matrix3f rotationInv = depthExtrinsicsInv.block(0, 0, 3, 3);
    Vector3f translationInv = depthExtrinsicsInv.block(0, 3, 3, 1);

    // Back-project the pixel depths into the camera space.
    std::vector<Vector3f> pointsTmp(width * height);

    // For every pixel row.
#pragma omp parallel for
    for (int v = 0; v < height; ++v)
    {
        // For every pixel in a row.
        for (int u = 0; u < width; ++u)
        {
            unsigned int idx = v * width + u; // linearized index
            float depth = depthMap[idx];
            if (depth == MINF)
            {
                pointsTmp[idx] = Vector3f(MINF, MINF, MINF);
            }
            else
            {
                // Back-projection to camera space.
                pointsTmp[idx] = rotationInv * Vector3f((u - cX) / fovX * depth, (v - cY) / fovY * depth, depth) + translationInv;
            }
        }
    }

    // We need to compute derivatives and then the normalized normal vector (for valid pixels).
    std::vector<Vector3f> normalsTmp(width * height);

#pragma omp parallel for
    for (int v = 1; v < height - 1; ++v)
    {
        for (int u = 1; u < width - 1; ++u)
        {
            unsigned int idx = v * width + u; // linearized index

            const float du = 0.5f * (depthMap[idx + 1] - depthMap[idx - 1]);
            const float dv = 0.5f * (depthMap[idx + width] - depthMap[idx - width]);
            if (!std::isfinite(du) || !std::isfinite(dv) || abs(du) > maxDistanceHalved || abs(dv) > maxDistanceHalved)
            {
                normalsTmp[idx] = Vector3f(MINF, MINF, MINF);
                continue;
            }

            // T0D0: Compute the normals using central differences.
            Vector3f c1 = pointsTmp[idx + 1] - pointsTmp[idx - 1];
            Vector3f c2 = pointsTmp[idx + width] - pointsTmp[idx - width];
            normalsTmp[idx] = -c1.cross(c2);
            normalsTmp[idx].normalize();
        }
    }

    // We set invalid normals for border regions.
    for (int u = 0; u < width; ++u)
    {
        normalsTmp[u] = Vector3f(MINF, MINF, MINF);
        normalsTmp[u + (height - 1) * width] = Vector3f(MINF, MINF, MINF);
    }
    for (int v = 0; v < height; ++v)
    {
        normalsTmp[v * width] = Vector3f(MINF, MINF, MINF);
        normalsTmp[(width - 1) + v * width] = Vector3f(MINF, MINF, MINF);
    }

    // We filter out measurements where either point or normal is invalid.
    const unsigned nPoints = pointsTmp.size();
    m_points.reserve(std::floor(float(nPoints) / downsampleFactor));
    m_normals.reserve(std::floor(float(nPoints) / downsampleFactor));

    for (int i = 0; i < nPoints; i = i + downsampleFactor)
    {
        const auto &point = pointsTmp[i];
        const auto &normal = normalsTmp[i];

        if (point.allFinite() && normal.allFinite())
        {
            m_points.push_back(point);
            m_normals.push_back(normal);
        }
    }
}

bool PointCloud::readFromFile(const std::string &filename)
{
    std::ifstream is(filename, std::ios::in | std::ios::binary);
    if (!is.is_open())
    {
        std::cout << "ERROR: unable to read input file!" << std::endl;
        return false;
    }

    char nBytes;
    is.read(&nBytes, sizeof(char));

    unsigned int n;
    is.read((char *)&n, sizeof(unsigned int));

    if (nBytes == sizeof(float))
    {
        float *ps = new float[3 * n];

        is.read((char *)ps, 3 * sizeof(float) * n);

        for (unsigned int i = 0; i < n; i++)
        {
            Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
            m_points.push_back(p);
        }

        is.read((char *)ps, 3 * sizeof(float) * n);
        for (unsigned int i = 0; i < n; i++)
        {
            Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
            m_normals.push_back(p);
        }

        delete[] ps;
    }
    else
    {
        double *ps = new double[3 * n];

        is.read((char *)ps, 3 * sizeof(double) * n);

        for (unsigned int i = 0; i < n; i++)
        {
            Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
            m_points.push_back(p);
        }

        is.read((char *)ps, 3 * sizeof(double) * n);

        for (unsigned int i = 0; i < n; i++)
        {
            Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
            m_normals.push_back(p);
        }

        delete[] ps;
    }

    // std::ofstream file("pointcloud.off");
    // file << "OFF" << std::endl;
    // file << m_points.size() << " 0 0" << std::endl;
    // for(unsigned int i=0; i<m_points.size(); ++i)
    //	file << m_points[i].x() << " " << m_points[i].y() << " " << m_points[i].z() << std::endl;
    // file.close();

    return true;
}

std::vector<Vector3f> &PointCloud::getPoints()
{
    return m_points;
}

const std::vector<Vector3f> &PointCloud::getPoints() const
{
    return m_points;
}

std::vector<Vector3f> &PointCloud::getNormals()
{
    return m_normals;
}

const std::vector<Vector3f> &PointCloud::getNormals() const
{
    return m_normals;
}

unsigned int PointCloud::getClosestPoint(Vector3f &p)
{
    unsigned int idx = 0;

    float min_dist = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < m_points.size(); ++i)
    {
        float dist = (p - m_points[i]).norm();
        if (min_dist > dist)
        {
            idx = i;
            min_dist = dist;
        }
    }

    return idx;
}