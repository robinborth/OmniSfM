#include <iostream>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <vector>
#include <stdexcept>

using std::vector;


struct Intrinsics;
struct Extrinsics;
struct Point2D;
struct Point3D;

struct ReprojectionErr;

void firstBundleAdjustmentTest(Intrinsics intrinsics,
                      Extrinsics extrinsics,
                      const vector<Point2D>& points2D_1,
                      const vector<Point2D>& points2D_2,
                      vector<Point3D>& points3D);
