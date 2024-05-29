#pragma once

#include <ceres/ceres.h>

class BundleAdjustmentConstraint
{
public:
    BundleAdjustmentConstraint() {}

    template <typename T>
    bool operator()(const T *const pose, T *residuals) const
    {
        // TODO: Implemented the bundle adjustment cost function.
        residuals[0] = T(0.0f);
        return true;
    }

    static ceres::CostFunction *create()
    {
        // TODO: Implement the cost function.
        return new ceres::AutoDiffCostFunction<BundleAdjustmentConstraint, 1, 1>(
            new BundleAdjustmentConstraint());
    }
};

class SfMOptimizer
{
public:
    SfMOptimizer() {}
};
