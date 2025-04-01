#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		// Clamp the absolute cosine value to [0,1]
		cosTheta = std::fabs(cosTheta);
		cosTheta = std::max(0.0f, std::min(cosTheta, 1.0f));
		// Compute base reflectance at normal incidence
		float r0 = (iorExt - iorInt) / (iorExt + iorInt);
		r0 = r0 * r0;
		// Use Schlick's approximation to compute the Fresnel term
		return r0 + (1.0f - r0) * std::pow(1.0f - cosTheta, 5.0f);
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		return Colour(1.0f, 1.0f, 1.0f);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		return 1.0f;
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		return 1.0f;
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		 // Convert the outgoing direction to the local (shading) frame.
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Compute the perfect reflection in local space.
		// For a local coordinate system where the shading normal is (0,0,1),
		// the reflection of woLocal = (x, y, z) is given by (-x, -y, z).
		Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);

		// Transform the reflection direction back to world space.
		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		// For a delta distribution, the PDF is defined in a way that
		// integration is done solely via sampling, so we can set it to 1.
		pdf = 1.0f;

		// The reflectance is given by the albedo texture sampled at the given UVs.
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		// Convert outgoing vector to local space (where the shading normal is (0,0,1)).
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		// In local space, the z-component is the cosine of the angle with the normal.
		float cosThetaI = woLocal.z;
		// Compute Fresnel reflectance for dielectrics.
		float F = ShadingHelper::fresnelDielectric(fabs(cosThetaI), intIOR, extIOR);
		// Choose event based on a uniform sample.
		float xi = sampler->next();
		Vec3 wi;
		if (xi < F) {
			// --- Reflection ---
			// In local space, perfect reflection is given by (−x, −y, z)
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
			wi = shadingData.frame.toWorld(wiLocal);
			// For a mirror, we assume the reflectance is given by the albedo (tint).
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
			// For delta functions we treat the discrete PDF as the event probability.
			pdf = F;
		}
		else {
			// --- Refraction ---
			// Determine the relative index of refraction.
			// If cosThetaI > 0, the ray is leaving the material.
			float eta = (cosThetaI > 0.0f) ? extIOR / intIOR : intIOR / extIOR;
			// Compute sin^2 theta_t using Snell's law.
			float sinThetaT2 = eta * eta * (1.0f - cosThetaI * cosThetaI);
			if (sinThetaT2 > 1.0f) {
				// Total internal reflection: treat as reflection.
				Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
				wi = shadingData.frame.toWorld(wiLocal);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
				pdf = 1.0f;
			}
			else {
				// Compute cosine of transmitted angle.
				float cosThetaT = sqrtf(1.0f - sinThetaT2);
				Vec3 wiLocal;
				// The sign of cosThetaT depends on the incident side.
				if (cosThetaI > 0.0f) {
					// Ray leaving: transmitted ray goes into the opposite hemisphere.
					wiLocal = Vec3(eta * -woLocal.x, eta * -woLocal.y, -cosThetaT);
				}
				else {
					// Ray entering: transmitted ray remains in the same hemisphere as the normal.
					wiLocal = Vec3(eta * -woLocal.x, eta * -woLocal.y, cosThetaT);
				}
				wi = shadingData.frame.toWorld(wiLocal);
				// The transmitted radiance is scaled by (1-F) and by eta^2 (to account for the change in area).
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1.0f - F) * (eta * eta);
				pdf = 1.0f - F;
			}
		}
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		// Transform the outgoing direction into the local frame.
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		// Compute Fresnel term (using Schlick's approximation).
		float cosTheta = fabsf(woLocal.z);
		float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);

		// Decide which lobe to sample: specular with probability F, diffuse with probability (1 - F)
		float xi = sampler->next();
		Vec3 wi;
		float pdfSpec = 0.0f, pdfDiff = 0.0f;
		if (xi < F) {
			// --- Specular (Glossy) Sampling using a Phong lobe ---
			float n = alphaToPhongExponent();
			// Sample the half-vector h from a cosine power (Phong) distribution.
			float r1 = sampler->next();
			float r2 = sampler->next();
			float phi = 2.0f * M_PI * r1;
			// In cosine-power sampling, the z component is:
			float cosThetaH = powf(r2, 1.0f / (n + 1.0f));
			float sinThetaH = sqrtf(1.0f - cosThetaH * cosThetaH);
			Vec3 h(sinThetaH * cosf(phi), sinThetaH * sinf(phi), cosThetaH);
			// Reflect woLocal about h:
			wi = -woLocal + h * 2.0f * Dot(woLocal, h);
			// Convert back to world space:
			wi = shadingData.frame.toWorld(wi);
			// Compute the PDF for the half-vector:
			float pdf_h = ((n + 1.0f) * powf(std::max(h.z, 0.0f), n)) / (2.0f * M_PI);
			// The mapping from half-vector to wi introduces a Jacobian factor: 1 / (4 * |Dot(woLocal, h)|)
			pdfSpec = pdf_h / (4.0f * fabsf(Dot(woLocal, h)) + 1e-6f);
			// For a glossy specular, we assume a white specular reflection (can be adjusted as needed)
			reflectedColour = Colour(1.0f, 1.0f, 1.0f);
			pdf = F * pdfSpec;
		}
		else {
			// --- Diffuse Sampling: Cosine-Weighted Hemisphere Sampling ---
			Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			wi = shadingData.frame.toWorld(wiLocal);
			// Diffuse component: Lambertian diffuse BRDF = albedo/π.
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
			pdfDiff = SamplingDistributions::cosineHemispherePDF(wiLocal);
			pdf = (1.0f - F) * pdfDiff;
		}
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		// Transform incoming direction to local space.
        Vec3 wiLocal = shadingData.frame.toLocal(wi);
        Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
        float cosTheta = fabsf(woLocal.z);
        // Compute Fresnel term.
        float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
        // Diffuse term (Lambertian): albedo / π.
        Colour diffuse = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
        // Specular term using a Phong lobe:
        float n = alphaToPhongExponent();
        // Compute half-vector h from wiLocal and woLocal:
        Vec3 h = (wiLocal + woLocal).normalize();
        float specularTerm = ((n + 2.0f) / (2.0f * M_PI)) * powf(std::max(h.z, 0.0f), n);
        Colour specular = Colour(1.0f, 1.0f, 1.0f) * specularTerm;
        // Return a weighted sum: specular is weighted by F, diffuse by (1-F).
        return diffuse * (1.0f - F) + specular * F;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = fabsf(woLocal.z);
		float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
		// Diffuse PDF:
		float pdfDiff = SamplingDistributions::cosineHemispherePDF(wiLocal);
		// Specular PDF: compute as in sample():
		float n = alphaToPhongExponent();
		Vec3 h = (wiLocal + woLocal).normalize();
		float pdf_h = ((n + 1.0f) * powf(std::max(h.z, 0.0f), n)) / (2.0f * M_PI);
		float pdfSpec = pdf_h / (4.0f * fabsf(Dot(woLocal, h)) + 1e-6f);
		return F * pdfSpec + (1.0f - F) * pdfDiff;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};