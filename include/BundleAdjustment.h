#pragma once

#include "SfmGraph.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>

class BundleAdjustment
{
public:
    void Adjust(SfMGraph &graph);

private:
    struct ReprojectionError
    {
        double observed_x;
        double observed_y;

        ReprojectionError(double observed_x, double observed_y)
            : observed_x(observed_x), observed_y(observed_y)
        {
            // std::cout << "ReprojectionError initialized with: " << observed_x << " " << observed_y << std::endl;
        }

        template <typename T>
        bool operator()(const T *const intrinsics, const T *const extrinsics, const T *const point, T *residuals) const
        {
            T p[3] = {T(0), T(0), T(0)};
            ceres::AngleAxisRotatePoint(extrinsics, point, p); // Rotate point
            // Translate point
            p[0] += extrinsics[3];
            p[1] += extrinsics[4];
            p[2] += extrinsics[5];

            // Perspective division to get normalized image coordinates
            T xp = p[0] / (p[2] + T(1e-10)); // zero div safety
            T yp = p[1] / (p[2] + T(1e-10));

            // Apply camera intrinsics
            T predicted_x = intrinsics[0] * xp + intrinsics[2]; // fx * xp + cx
            T predicted_y = intrinsics[1] * yp + intrinsics[3]; // fy * yp + cy
            // std::cout << "Predicted: " << predicted_x << ", " << predicted_y << std::endl;
            // std::cout << "Observed: " << observed_x << ", " << observed_y << std::endl;

            // Compute residuals
            residuals[0] = predicted_x - T(observed_x);
            residuals[1] = predicted_y - T(observed_y);

            return true;
        }
    };

    static ceres::CostFunction *CreateCostFunction(double observed_x, double observed_y)
    {
        return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 6, 3>(
            new ReprojectionError(observed_x, observed_y));
    }

    Eigen::Matrix<double, 6, 1> extractExtrinsics(const Eigen::Matrix4f &pose);
    Eigen::Matrix<double, 4, 1> extractIntrinsics(const Eigen::Matrix3f &matrix);
    Eigen::Matrix<double, 3, 1> extractPoint3d(const Eigen::Vector4f &position);
    std::map<int, Eigen::Matrix<double, 6, 1>> extractAllExtrinsics(const SfMGraph &graph);
    std::vector<Eigen::Matrix<double, 3, 1>> extractAllPoint3d(const SfMGraph &graph);
    std::vector<Eigen::Matrix4f> constructPoseFromExtrinsics(const std::map<int, Eigen::Matrix<double, 6, 1>> &extrinsicsMap);
    std::vector<Vertex> construct3dPoints(const std::vector<Eigen::Matrix<double, 3, 1>> &point3ds, const SfMGraph &graph);
};