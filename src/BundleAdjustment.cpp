#include "BundleAdjustment.h"
#include "Visualization.h"
#include <iostream>

std::map<int, Eigen::Matrix<double, 6, 1>> BundleAdjustment::extractAllExtrinsics(const SfMGraph &graph)
{
    std::map<int, Eigen::Matrix<double, 6, 1>> extrinsics;
    for (const auto &cam : graph.cams)
    {
        extrinsics[cam.id] = extractExtrinsics(cam.pose);
    }
    std::cout << "Extracted all extrinsics" << extrinsics.size() << std::endl;
    return extrinsics;
}

Eigen::Matrix<double, 6, 1> BundleAdjustment::extractExtrinsics(const Eigen::Matrix4f &pose)
{
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
    for (int i = 0; i < 3; i++)
    {
        pointArr(i) = position(i);
    }
    return pointArr;
}

std::vector<Eigen::Matrix<double, 3, 1>> BundleAdjustment::extractAllPoint3d(const SfMGraph &graph)
{
    std::vector<Eigen::Matrix<double, 3, 1>> point3dArr;
    for (const auto &point3d : graph.point3DList)
    {
        point3dArr.push_back(extractPoint3d(point3d.position));
    }
    return point3dArr;
}

std::vector<Eigen::Matrix4f> BundleAdjustment::constructPoseFromExtrinsics(const std::map<int, Eigen::Matrix<double, 6, 1>> &extrinsicsMap)
{
    std::vector<Eigen::Matrix4f> poses;
    for (const auto &extrinsic : extrinsicsMap)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

        // Extract the rotation vector and translation vector from the current extrinsics element
        Eigen::Matrix<double, 3, 1> rotation_vec = extrinsic.second.head<3>();
        Eigen::Matrix<double, 3, 1> translation_vec = extrinsic.second.tail<3>();

        Eigen::Matrix3d rotationMatrix;
        ceres::AngleAxisToRotationMatrix(rotation_vec.data(), rotationMatrix.data());

        // Cast the rotation matrix from double to float and insert it into the pose matrix
        pose.block<3, 3>(0, 0) = rotationMatrix.cast<float>();

        // Cast the translation vector from double to float and insert it into the pose matrix
        pose.block<3, 1>(0, 3) = translation_vec.cast<float>();

        poses.push_back(pose);
    }

    return poses;
}

std::vector<Vertex> BundleAdjustment::construct3dPoints(const std::vector<Eigen::Matrix<double, 3, 1>> &point3ds, const SfMGraph &graph)
{
    std::vector<Vertex> points3D;
    for (size_t i = 0; i < point3ds.size(); i++)
    {
        Eigen::Matrix<double, 3, 1> point3d = point3ds[i];
        Point3D point = graph.point3DList[i];
        Vertex vertex;
        vertex.position = Eigen::Vector4f(point3d(0), point3d(1), point3d(2), 1.0);
        vertex.color = point.color;
        points3D.push_back(vertex);
    }
    return points3D;
}

void BundleAdjustment::Adjust(SfMGraph &graph)
{
    ceres::Problem problem;
    Eigen::Matrix<double, 4, 1> intrinsicsArr = extractIntrinsics(graph.cams[1].intrinsics); // same intrinsics for all cameras
    std::map<int, Eigen::Matrix<double, 6, 1>> extrinsics = extractAllExtrinsics(graph);
    std::vector<Eigen::Matrix<double, 3, 1>> point3ds = extractAllPoint3d(graph); // ensure no duplicates

    for (size_t j = 0; j < graph.point3DList.size(); j++)
    {
        Eigen::Matrix<double, 3, 1> point3dArr = point3ds[j];
        for (size_t i = 0; i < graph.point3DList[j].observations.size(); i++)
        {
            std::pair<int, int> observation = graph.point3DList[j].observations[i];
            int nodeIdx = observation.first;
            Node cam = graph.getNode(nodeIdx);
            Eigen::Matrix<double, 6, 1> extrinsicsArr = extrinsics[nodeIdx];
            double observed_x = cam.keypoints[observation.second].pt.x;
            double observed_y = cam.keypoints[observation.second].pt.y;
            ceres::CostFunction *cost_function = CreateCostFunction(observed_x, observed_y);
            // std::cout << "############################################################" << std::endl;
            problem.AddResidualBlock(cost_function, NULL, intrinsicsArr.data(), extrinsicsArr.data(), point3dArr.data());

            problem.SetParameterBlockConstant(intrinsicsArr.data());
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.max_num_iterations = 100;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.minimizer_progress_to_stdout = true;

    options.use_nonmonotonic_steps = true;
    options.function_tolerance = 1e-10;
    options.max_trust_region_radius =10000;
    options.min_trust_region_radius = 1e-2;
    ceres::Solver::Summary summary;
    std::cout << "Solving..." << std::endl;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // Here after the optimization, we need to get the updated extrinsics and 3D points
    // TODO : write these outputs to the graph after verifying them
    std::vector<Vertex> points3D = construct3dPoints(point3ds, graph);
    std::cout << " Size of points3D: " << points3D.size() << std::endl;
    std::vector<Eigen::Matrix4f> cameraPoses = constructPoseFromExtrinsics(extrinsics);

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
    sfmVis.addCamera(cameraPoses, 0.001);
    sfmVis.writeAllMeshes();
}
