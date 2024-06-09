#pragma once
#include "Eigen.h"

struct Match
{
	int id;
	float weight;
};

template <int FeatureSize>
class NearestNeighborSearch
{
	using Feature = Eigen::Matrix<float, FeatureSize, 1>;

public:
	NearestNeighborSearch() : threshold{0.9f} {}

	void buildIndex(const std::vector<Feature> &targetPoints)
	{
		points = targetPoints;
	}

	void setThreshold(float threshold)
	{
		this->threshold = threshold;
	}

	std::vector<Match> queryMatches(const std::vector<Feature> &transformedPoints)
	{
		const unsigned nMatches = transformedPoints.size();
		std::vector<Match> matches(nMatches);
		const unsigned nTargetPoints = points.size();
		for (int i = 0; i < nMatches; i++)
		{
			matches[i] = getClosestPoint(transformedPoints[i]);
		}
		return matches;
	}

private:
	std::vector<Feature> points;
	float threshold;

	Match getClosestPoint(const Feature &p)
	{
		int idx = -1;
		float minDist = std::numeric_limits<float>::max();
		for (unsigned int i = 0; i < points.size(); ++i)
		{
			float dist = (p - points[i]).norm();
			if (minDist > dist)
			{
				idx = i;
				minDist = dist;
			}
		}
		if (idx == -1)
			return Match{idx, 0.0f};
		return Match{idx, minDist};
	}
};