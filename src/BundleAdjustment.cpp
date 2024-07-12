#include "BundleAdjustment.h"
#include <iostream>

void BundleAdjustment::Adjust(SfMGraph& graph) {
    ceres::Problem problem;

    // Set up the problem by adding camera parameters and points to the solver
    AddObservationsToProblem(graph, problem);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    std::cout << "Solving..." << std::endl;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
}

Eigen::Matrix<double, 6, 1> BundleAdjustment::extractExtrinsics(const Eigen::Matrix4f &pose) {
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

Eigen::Matrix<double, 4, 1> BundleAdjustment::extractIntrinsics(const Eigen::Matrix3f &matrix) 
{
    Eigen::Matrix<double, 4, 1> intrinsics; // for fx, fy, cx, cy
    // cast matrix to double
    Eigen::Matrix<double, 3, 3> matrix_double = matrix.cast<double>();

    intrinsics(0) = matrix_double(0, 0); // fx
    intrinsics(1) = matrix_double(1, 1); // fy
    intrinsics(2) = matrix_double(0, 2); // cx
    intrinsics(3) = matrix_double(1, 2); // cy

    return intrinsics;
}

Eigen::Matrix<double, 3, 1> BundleAdjustment::extractPoint3d(const Eigen::Vector4f &position) 
{
    Eigen::Matrix<double, 3, 1> pointArr;
    for (int i = 0; i < 3; i++) {
        pointArr(i) = position(i);
    }
    return pointArr;
}

void BundleAdjustment::AddObservationsToProblem(SfMGraph &graph, ceres::Problem &problem) 
{
    Eigen::Matrix<double, 4, 1> intrinsicsArr = extractIntrinsics(graph.cams[1].intrinsics); // same intrinsics for all cameras

    for (const auto &point3d : graph.point3DList) 
    {
        Eigen::Matrix<double, 3, 1> point3dArr = extractPoint3d(point3d.position);
        for (size_t i = 0; i < point3d.observations.size(); i++) 
        {
            int nodeIdx = point3d.observations[i].first;
            Node cam = graph.getNode(nodeIdx);
            Eigen::Matrix<double, 6, 1> extrinsicsArr = extractExtrinsics(cam.pose);
            double observed_x = cam.keypoints[point3d.observations[i].second].pt.x;
            double observed_y = cam.keypoints[point3d.observations[i].second].pt.y;
            ceres::CostFunction *cost_function = CreateCostFunction(observed_x, observed_y);

            problem.AddResidualBlock(cost_function, NULL, intrinsicsArr.data(), extrinsicsArr.data(), point3dArr.data());
        }
    }
}
