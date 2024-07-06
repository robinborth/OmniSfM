#include <iostream>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include <stdexcept>

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::runtime_error;

struct Point2D {
    float x;
    float y;
};

struct Point3D {
    float x;
    float y;
    float z;
};

struct Intrinsics {
    float fX;
    float fY;
    float cX;
    float cY;
};

struct Extrinsics {
    float tx;
    float ty;
    float tz;
    float qx;
    float qy;
    float qz;
    float qw;
};

struct ReprojectionErr {
    Point2D point2D;

    ReprojectionErr(const Point2D& point2D): point2D(point2D) {}

    template <typename T>
    bool operator()(const T* const intrinsics,
                    const T* const extrinsics,
                    const T* const point3D,
                    T* residuals) const {

        const T& fX = intrinsics[0];
        const T& fY = intrinsics[1];
        const T& cX = intrinsics[2];
        const T& cY = intrinsics[3];
        const T* translation = extrinsics;
        const T* rotation = extrinsics + 3;

        // to camera coords
        T point3DHat[3];
        point3DHat[0] += translation[0];
        point3DHat[1] += translation[1];
        point3DHat[2] += translation[2];
        ceres::QuaternionRotatePoint(rotation, point3D, point3DHat);

        // to image
        T point2DXHat = fX * (point3DHat[0] / point3DHat[2]) + cX;
        T point2DYHat = fY * (point3DHat[1] / point3DHat[2]) + cY;

        residuals[0] = point2DXHat - T(point2D.x);
        residuals[1] = point2DYHat - T(point2D.y);

        return true;
    }
};

void firstBundleAdjustmentTest(Intrinsics intrinsics,
                      Extrinsics extrinsics,
                      const vector<Point2D>& points2D_1,
                      const vector<Point2D>& points2D_2,
                      vector<Point3D>& points3D) {

    size_t numPoints = points2D_1.size();
    if (numPoints != points2D_2.size() || numPoints != points3D.size()) {
        throw runtime_error("Number of points not consistent.");
    }

    // ceres needs doubles
    double intrinsicsArr[4] = { intrinsics.fX, intrinsics.fY, intrinsics.cX, intrinsics.cY };
    double extrinsicsArr[7] = { extrinsics.tx, extrinsics.ty, extrinsics.tz, extrinsics.qx, extrinsics.qy, extrinsics.qz, extrinsics.qw };

    ceres::Problem problem;
    problem.AddParameterBlock(intrinsicsArr, 4);
    problem.AddParameterBlock(extrinsicsArr, 7);

    for (size_t i; i < numPoints; i++) {
        double* point3DArr = reinterpret_cast<double*>(&points3D[i]);
        problem.AddParameterBlock(point3DArr, 3);
        
        // 2: residuals, 4: 1st para block (intrinsics), 7: 2nd para block (extrinsics), 3: 3rd para block (3D point)
        ceres::CostFunction* costFn = new ceres::AutoDiffCostFunction<ReprojectionErr, 2, 4, 7, 3>(
            new ReprojectionErr(points2D_1[i])
        );
        problem.AddResidualBlock(costFn, nullptr, intrinsicsArr, extrinsicsArr, point3DArr);

        costFn = new ceres::AutoDiffCostFunction<ReprojectionErr, 2, 4, 7, 3>(
            new ReprojectionErr(points2D_2[i])
        );
        problem.AddResidualBlock(costFn, nullptr, intrinsicsArr, extrinsicsArr, point3DArr);
    }

    // http://ceres-solver.org/nnls_solving.html
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.FullReport() << endl;
}