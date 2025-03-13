#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float phi = 2.0f * M_PI * r2;    // azimuth in [0, 2π)
		float cosTheta = 1.0f - r1;          // from the CDF
		float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);

		// 2) Convert spherical (theta, phi) to Cartesian (x, y, z).
		float x = sinTheta * cosf(phi);
		float y = sinTheta * sinf(phi);
		float z = cosTheta; // points "up" since z >= 0

		return Vec3(x, y, z);
		//return Vec3(0, 0, 1);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return (wi.z >= 0.0f) ? 1.0f / (2.0f * M_PI) : 0.0f;
		//return 1.0f;
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		// Interpret r1 as the radial distance squared on a unit disk.
		float r = sqrtf(r1);
		float theta = 2.0f * M_PI * r2;

		// Convert polar coordinates to Cartesian for the disk (x,y).
		float x = r * cosf(theta);
		float y = r * sinf(theta);

		// The z-coordinate is derived from z = sqrt(1 - r^2).
		float z = sqrtf(std::max(0.0f, 1.0f - r1));

		return Vec3(x, y, z);
		//return Vec3(0, 0, 1);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		// The PDF is (z / π) for z ≥ 0, otherwise 0.
		return (wi.z > 0.0f) ? wi.z / M_PI : 0.0f;
		//return 1.0f;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		// Map r1 to z in [-1,1].
		float z = 1.0f - 2.0f * r1;
		float phi = 2.0f * M_PI * r2; // azimuth in [0, 2π)

		// Compute the radius of the circle at this z.
		float r = sqrtf(std::max(0.0f, 1.0f - z * z));

		// Convert to Cartesian coordinates.
		float x = r * cosf(phi);
		float y = r * sinf(phi);

		return Vec3(x, y, z);
		//return Vec3(0, 0, 1);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f / (4.0f * M_PI);
		//return 1.0f;
	}
};