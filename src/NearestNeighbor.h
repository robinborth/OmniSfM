#pragma once
#include "Eigen.h"

struct Match
{
	int sourceId;
	int targetId;
	float weight;
};

template <int FeatureSize>
using Feature = Eigen::Matrix<float, FeatureSize, 1>;

template <int FeatureSize>
class NearestNeighborSearch
{
	using FeatureT = Feature<FeatureSize>;

public:
	NearestNeighborSearch() : threshold{0.9f} {}

	void buildIndex(const std::vector<FeatureT> &targetPoints)
	{
		points = targetPoints;
	}

	void buildIndex(const Image &targetImage)
	{
		points = matToVector(targetImage.descriptors);
	}

	void setThreshold(float threshold)
	{
		this->threshold = threshold;
	}

	std::vector<Match> queryMatches(const std::vector<FeatureT> &sourcePoints)
	{
		std::vector<Match> matches;
		for (size_t i = 0; i < sourcePoints.size(); ++i)
		{
			Match match = getClosestPoint(sourcePoints[i]);
			if (match.targetId != -1)
			{
				match.sourceId = i;
				matches.push_back(match);
			}
		}
		return matches;
	}

	std::vector<Match> queryMatches(const Image &sourceImage)
	{
		return queryMatches(matToVector(sourceImage.descriptors));
	}

private:
	std::vector<FeatureT> points;
	float threshold;

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
		match.targetId = idx;
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