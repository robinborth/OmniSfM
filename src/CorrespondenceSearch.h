#pragma once

#include "Eigen.h"

struct Match
{
	int sourceImageId;
	int targetImageId;
	int sourceKeypointId;
	int targetKeyopintId;
	float weight;
};

template <int FeatureSize>
using Feature = Eigen::Matrix<float, FeatureSize, 1>;

template <int FeatureSize>
class CorrespondenceSearch
{
	using FeatureT = Feature<FeatureSize>;

public:
	CorrespondenceSearch() : threshold{0.9f} {}

	void setThreshold(float threshold)
	{
		this->threshold = threshold;
	}

	std::vector<Match> queryMatches(const Image &sourceImage, const Image &targetImage)
	{
		// build the index to query from, this is the points state of the class
		buildIndex(targetImage);

		// prepare the source points from the source image
		auto sourcePoints = matToVector(sourceImage.descriptors);

		// fine the matches and return them
		std::vector<Match> matches;
		for (size_t i = 0; i < sourcePoints.size(); ++i)
		{
			Match match = getClosestPoint(sourcePoints[i]);
			if (match.targetKeyopintId != -1)
			{
				match.sourceKeypointId = i;
				match.targetImageId = targetImage.id;
				match.sourceImageId = sourceImage.id;
				matches.push_back(match);
			}
		}
		return matches;
	}

	std::vector<Match> queryCorrespondences(std::vector<Image> images)
	{
		// TODO implement correspondences
		return {};
	}

private:
	std::vector<FeatureT> points;
	float threshold;

	void buildIndex(const Image &targetImage)
	{
		points = matToVector(targetImage.descriptors);
	}

	Match getClosestPoint(const FeatureT &p)
	{
		// return the cosine similarity as weight
		int idx = -1;
		float minDist = std::numeric_limits<float>::max();
		float maxSimilarity = -1.0;
		for (unsigned int i = 0; i < points.size(); ++i)
		{
			float similarity = p.dot(points[i]) / (p.norm() * points[i].norm());
			if (maxSimilarity < similarity && similarity > threshold)
			{
				idx = i;
				maxSimilarity = similarity;
			}
		}
		// build the match for the target, missing source id
		Match match;
		match.targetKeyopintId = idx;
		match.weight = maxSimilarity;
		if (idx == -1)
			match.weight = 0;
		return match;
	}

	std::vector<FeatureT> matToVector(const cv::Mat &mat)
	{
		std::vector<FeatureT> vec;
		vec.reserve(mat.rows);

		for (int i = 0; i < mat.rows; ++i)
		{
			FeatureT feature;
			for (int j = 0; j < FeatureSize; ++j)
			{
				feature(j, 0) = mat.at<float>(i, j);
			}
			vec.push_back(feature);
		}
		return vec;
	}
};