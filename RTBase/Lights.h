﻿#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLightLT(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wi = Vec3(0, 0, -1); // 0 0 1
		pdf = 1.0f;
		/*Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);*/
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
	Vec3 sampleDirectionFromLightLT(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		//Vec3 wi = Vec3(0, 0, -1); // 0 0 1
		//pdf = 1.0f;
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
	Vec3 sampleDirectionFromLightLT(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

inline float clamp(float x, float lower, float upper) {
	return (x < lower) ? lower : (x > upper ? upper : x);
}

class EnvironmentMap : public Light
{
public:
	Texture* env;
	// Precomputed distribution arrays:
	std::vector<float> marginalCDF;                 
	std::vector<std::vector<float>> conditionalCDF;   
	float totalWeight;
	bool distributionBuilt;

	EnvironmentMap(Texture* _env)
	{
		env = _env;
		distributionBuilt = false;
		precomputeDistribution();
	}

	// Precompute the 2D distribution from the environment map.
	void precomputeDistribution()
	{
		int width = env->width;
		int height = env->height;
		marginalCDF.resize(height);
		conditionalCDF.resize(height);
		totalWeight = 0.0f;
		for (int v = 0; v < height; v++) {
			conditionalCDF[v].resize(width);
			float rowSum = 0.0f;
			float theta = M_PI * ((v + 0.5f) / height);
			float sinTheta = sinf(theta);
			for (int u = 0; u < width; u++) {
				Colour c = env->texels[v * width + u];

				float weight = c.Lum() * sinTheta;
				conditionalCDF[v][u] = weight;
				rowSum += weight;
			}
			// Build the prefix sum for the row:
			for (int u = 1; u < width; u++) {
				conditionalCDF[v][u] += conditionalCDF[v][u - 1];
			}
			marginalCDF[v] = rowSum;
			totalWeight += rowSum;
		}
		// Build the marginal CDF over rows:
		for (int v = 1; v < height; v++) {
			marginalCDF[v] += marginalCDF[v - 1];
		}
		distributionBuilt = true;
	}

	// Importance sample the environment map.
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		int width = env->width;
		int height = env->height;
		// Generate two uniform random numbers.
		float xi1 = sampler->next();
		float xi2 = sampler->next();

		// Sample the marginal distribution to choose a row (v).
		float target = xi1 * marginalCDF[height - 1];
		int v = std::lower_bound(marginalCDF.begin(), marginalCDF.end(), target) - marginalCDF.begin();

		// Determine the weight.
		float rowWeight = (v == 0) ? marginalCDF[0] : (marginalCDF[v] - marginalCDF[v - 1]);

		// Sample the conditional distribution
		float targetRow = xi2 * rowWeight;
		int u = std::lower_bound(conditionalCDF[v].begin(), conditionalCDF[v].end(), targetRow) - conditionalCDF[v].begin();

		// Convert to normalized coordinates
		float uCoord = (u + 0.5f) / float(width);
		float vCoord = (v + 0.5f) / float(height);

		// Map to spherical coordinates
		float phi = uCoord * 2.0f * M_PI;
		float theta = vCoord * M_PI;
		float sinTheta = sinf(theta);
		float cosTheta = cosf(theta);

		// construct direction (y up)
		Vec3 wi(sinTheta * cosf(phi), cosTheta, sinTheta * sinf(phi));

		// Get the radiance from the environment map:
		reflectedColour = evaluate(wi);

		// Compute the PDF
		float pixelWeight = (u == 0 ? conditionalCDF[v][0] : conditionalCDF[v][u] - conditionalCDF[v][u - 1]);
		float p_uv = (pixelWeight / totalWeight); 
		pdf = p_uv * (width * height) / (2.0f * M_PI * M_PI * sinTheta + 1e-6f);

		return wi;

		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;*/
	}

	// Evaluate the environment map: convert a direction to (u, v) coordinates and look up the radiance.
	Colour evaluate(const Vec3& wi)
	{
		// Convert the 3D direction to spherical coordinates.
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0.0f)
			phi += 2.0f * M_PI;
		float theta = acosf(clamp(wi.y, -1.0f, 1.0f));
		float u = phi / (2.0f * M_PI);
		float v = theta / M_PI;
		
		Colour envRadiance = env->sample(u, v);
		// apply exposure: reduce brightness
		float exposure = 0.2f; 
		return envRadiance * exposure;
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		//return SamplingDistributions::uniformSpherePDF(wi);
		int width = env->width;
		int height = env->height;
		float phi = atan2f(wi.z, wi.x);
		if (phi < 0.0f)
			phi += 2.0f * M_PI;
		float theta = acosf(clamp(wi.y, -1.0f, 1.0f));
		float sinTheta = sinf(theta);
		// Map (phi, theta) to texture coordinates.
		float uCoord = phi / (2.0f * M_PI);
		float vCoord = theta / M_PI;
		// Determine pixel indices.
		int u = std::min(width - 1, std::max(0, int(uCoord * width)));
		int v = std::min(height - 1, std::max(0, int(vCoord * height)));
		// Retrieve the weight for that pixel.
		Colour tex = env->texels[v * width + u];
		float weight = tex.Lum() * sinTheta;
		
		return (weight / totalWeight) / (2.0f * M_PI * sinTheta + 1e-6f);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;*/
		Colour dummy;
		return sample(ShadingData(), sampler, dummy, pdf);
	}
	Vec3 sampleDirectionFromLightLT(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		/*Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;*/
		Colour dummy;
		return sample(ShadingData(), sampler, dummy, pdf);
	}
};

