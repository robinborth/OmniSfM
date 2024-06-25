#pragma once

#include <stdint.h>
#include <opencv2/core.hpp>
#include "Eigen.h"

struct Match
{
	int64_t sourceImageId;
	int64_t targetImageId;
	int sourceKeypointId;
	int targetKeyopintId;
	float weight;
};

struct Image
{
	int64_t id;
	// rgb settings
	cv::Mat rgb;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	cv::Mat R; // world to camera
	cv::Mat t;
	Eigen::Matrix4f P;	   // depth information
	Eigen::Matrix3f K;	   // intrinsics matrix
	Eigen::MatrixXf depth; // depth information
	float q;			   // shift factor
	float w;			   // scale factor
};

struct ColoredPoint3f
{
	cv::Point3f point;
	cv::Vec3b color;
};

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Position stored as 4 floats (4th component is supposed to be 1.0)
	Eigen::Vector4f position;
	// Color stored as 4 unsigned char
	Vector4uc color;
};

struct Triangle
{
	unsigned int idx0;
	unsigned int idx1;
	unsigned int idx2;

	Triangle() : idx0{0}, idx1{0}, idx2{0} {}

	Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) : idx0(_idx0), idx1(_idx1), idx2(_idx2) {}
};