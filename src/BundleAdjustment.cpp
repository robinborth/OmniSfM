#include "BundleAdjustment.h"
#include "Visualization.h"
#include <iostream>

void BundleAdjustment::Adjust(SfMGraph &graph)
{
    ceres::Problem problem;
    Eigen::Matrix<double, 4, 1> intrinsicsArr = graph.extractIntrinsics(); // same intrinsics for all cameras
    std::map<int, Eigen::Matrix<double, 6, 1>> extrinsics = graph.extractAllExtrinsics();
    std::vector<Eigen::Matrix<double, 3, 1>> point3ds = graph.extractAllPoint3d(); // ensure no duplicates
    //ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);

    for (size_t j = 0; j < graph.point3DList.size(); j++)
    {
        for (size_t i = 0; i < graph.point3DList[j].observations.size(); i++)
        {
            std::pair<int, int> observation = graph.point3DList[j].observations[i];
            int nodeIdx = observation.first;
            Node cam = graph.getNode(nodeIdx);
            double observed_x = cam.keypoints[observation.second].pt.x;
            double observed_y = cam.keypoints[observation.second].pt.y;
            ceres::CostFunction *cost_function = CreateCostFunction(observed_x, observed_y);
            // std::cout << "############################################################" << std::endl;
            problem.AddResidualBlock(cost_function, loss_function, intrinsicsArr.data(), extrinsics[nodeIdx].data(), point3ds[j].data());

            problem.SetParameterBlockConstant(intrinsicsArr.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.max_num_iterations = 100;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    options.use_nonmonotonic_steps = false;
    options.function_tolerance = 1e-6;
    options.max_trust_region_radius =10000;
    options.min_trust_region_radius = 1e-3;
    ceres::Solver::Summary summary;
    std::cout << "Solving..." << std::endl;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // Here after the optimization, we need to get the updated extrinsics and 3D points
    // TODO : write these outputs to the graph after verifying them
    std::vector<Vertex> points3D = graph.construct3dPoints(point3ds);
    std::cout << " Size of points3D: " << points3D.size() << std::endl;
    std::vector<Eigen::Matrix4f> cameraPoses = graph.constructPoseFromExtrinsics(extrinsics);

    graph.updateAdjusted3DPointPoses(points3D);
    //graph.updateAdjustedIntrinsicParams(intrinsicsArr);
    graph.updateAdjustedExtrinsicParams(cameraPoses);

    std::cout << intrinsicsArr(0) << " " << intrinsicsArr(1) << " " << intrinsicsArr(2) << " " << intrinsicsArr(3) << std::endl;

    for (size_t i = 0; i < cameraPoses.size(); ++i)
    {
        std::cout << "Camera Pose " << i + 1 << ":\n";
        std::cout << cameraPoses[i] << "\n\n";
    }

    std::cout << "==> Visualize Bundle Adjustment ..." << std::endl;
    Visualization sfmVis = Visualization("ba");
    sfmVis.addVertex(points3D);
    sfmVis.addCamera(cameraPoses, 0.0002);
    sfmVis.writeAllMeshes();
}
