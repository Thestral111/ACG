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
		cosTheta = std::fabs(cosTheta);
		// Compute squares
		Colour eta2 = ior * ior;
		Colour k2 = k * k;
		
		Colour twoEtaCos = ior * (2.0f * cosTheta);
		float cosTheta2 = cosTheta * cosTheta;

		// Rs: parallel polarization
		Colour Rs_num = (eta2 + k2) * Colour(cosTheta2, cosTheta2, cosTheta2) - twoEtaCos + Colour(1.0f, 1.0f, 1.0f);
		Colour Rs_den = (eta2 + k2) * Colour(cosTheta2, cosTheta2, cosTheta2) + twoEtaCos + Colour(1.0f, 1.0f, 1.0f);
		Colour Rs = Rs_num / Rs_den;

		// Rp: perpendicular polarization
		Colour Rp_num = (eta2 + k2) - twoEtaCos + Colour(cosTheta2, cosTheta2, cosTheta2);
		Colour Rp_den = (eta2 + k2) + twoEtaCos + Colour(cosTheta2, cosTheta2, cosTheta2);
		Colour Rp = Rp_num / Rp_den;

		return (Rs + Rp) * 0.5f;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		float cosTheta = wi.z;
		if (cosTheta <= 0.0f)
			return 0.0f;
		float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		float tanTheta = sinTheta / cosTheta;
		float a2Tan2 = (alpha * alpha) * (tanTheta * tanTheta);
		return (sqrtf(1.0f + a2Tan2) - 1.0f) / 2.0f;
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		float lambda_i = lambdaGGX(wi, alpha);
		float lambda_o = lambdaGGX(wo, alpha);
		return 1.0f / (1.0f + lambda_i + lambda_o);
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		float cosThetaH = std::max(h.z, 0.0f);
		float cosThetaH2 = cosThetaH * cosThetaH;
		float alpha2 = alpha * alpha;
		float denom = (cosThetaH2 * (alpha2 - 1.0f) + 1.0f);
		denom = M_PI * denom * denom;
		return alpha2 / denom;
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
		// Convert to local
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Compute the perfect reflection
		Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);

		// Transform the reflection direction back to world space
		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		pdf = 1.0f;

		// The reflectance
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
		
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

		// Sample the half-vector h in local space using GGX distribution
		float r1 = sampler->next();
		float r2 = sampler->next();
		float phi = 2.0f * M_PI * r1;
		// Inversion for GGX:
		float tanTheta2 = (alpha * alpha * r2) / (1.0f - r2);
		float cosTheta_h = 1.0f / sqrtf(1.0f + tanTheta2);
		float sinTheta_h = sqrtf(std::max(0.0f, 1.0f - cosTheta_h * cosTheta_h));
		Vec3 hLocal(sinTheta_h * cosf(phi), sinTheta_h * sinf(phi), cosTheta_h);

		// Reflect woLocal about hLocal: wiLocal = reflect(-woLocal, hLocal)
		Vec3 wiLocal = -woLocal + hLocal * (2.0f * Dot(woLocal, hLocal));
		// Ensure the sampled wi is above the surface
		if (wiLocal.z <= 0)
		{
			pdf = 0.0f;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		// Transform the sampled direction back to world space
		Vec3 wi = shadingData.frame.toWorld(wiLocal);

		// Compute Fresnel term for conductors
		float cosTheta = fabsf(Dot(wiLocal, hLocal));
		Colour F = ShadingHelper::fresnelConductor(cosTheta, eta, k);

		// Compute the GGX normal distribution function (D) for h
		float D = ShadingHelper::Dggx(hLocal, alpha);

		// Compute the geometry (masking-shadowing) term G
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);

		// Denominator
		float denom = 4.0f * fabsf(woLocal.z) * fabsf(wiLocal.z) + 1e-6f;

		// Microfacet BRDF value
		Colour brdf = F * D * G / denom;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		// Compute PDF
		float pdf_h = D * hLocal.z;
		pdf = pdf_h / (4.0f * fabsf(Dot(woLocal, hLocal)) + 1e-6f);

		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		// Compute the half-vector h.
		Vec3 hLocal = (wiLocal + woLocal).normalize();
		float cosTheta = fabsf(Dot(wiLocal, hLocal));
		Colour F = ShadingHelper::fresnelConductor(cosTheta, eta, k);
		float D = ShadingHelper::Dggx(hLocal, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float denom = 4.0f * fabsf(woLocal.z) * fabsf(wiLocal.z) + 1e-6f;
		Colour brdf = F * D * G / denom;
		Colour tint = albedo->sample(shadingData.tu, shadingData.tv);
		return brdf * tint;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 hLocal = (wiLocal + woLocal).normalize();
		float D = ShadingHelper::Dggx(hLocal, alpha);
		float pdf_h = D * hLocal.z;
		return pdf_h / (4.0f * fabsf(Dot(woLocal, hLocal)) + 1e-6f);
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
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosThetaI = woLocal.z;
		// Compute Fresnel reflectance
		float F = ShadingHelper::fresnelDielectric(fabs(cosThetaI), intIOR, extIOR);
		// Choose event based on a uniform sample
		float xi = sampler->next();
		Vec3 wi;
		if (xi < F) {
			// --- Reflection ---
			Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
			wi = shadingData.frame.toWorld(wiLocal);
			// like mirror
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

			pdf = F;
		}
		else {
			// --- Refraction ---
			
			float eta = (cosThetaI > 0.0f) ? extIOR / intIOR : intIOR / extIOR;
			// Snell's law.
			float sinThetaT2 = eta * eta * (1.0f - cosThetaI * cosThetaI);
			if (sinThetaT2 > 1.0f) {
				// total internal reflection
				Vec3 wiLocal(-woLocal.x, -woLocal.y, woLocal.z);
				wi = shadingData.frame.toWorld(wiLocal);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
				pdf = 1.0f;
			}
			else {

				float cosThetaT = sqrtf(1.0f - sinThetaT2);
				Vec3 wiLocal;

				if (cosThetaI > 0.0f) {
					// ray leaveing
					wiLocal = Vec3(eta * -woLocal.x, eta * -woLocal.y, -cosThetaT);
				}
				else {
					// ray entering
					wiLocal = Vec3(eta * -woLocal.x, eta * -woLocal.y, cosThetaT);
				}
				wi = shadingData.frame.toWorld(wiLocal);

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

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		// Compute Fresnel
		float cosTheta = fabsf(woLocal.z);
		float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);


		float xi = sampler->next();
		Vec3 wi;
		float pdfSpec = 0.0f, pdfDiff = 0.0f;
		if (xi < F) {
			// glossy phong reflection
			float n = alphaToPhongExponent();
			float r1 = sampler->next();
			float r2 = sampler->next();
			float phi = 2.0f * M_PI * r1;
			// cosine-power sampling
			float cosThetaH = powf(r2, 1.0f / (n + 1.0f));
			float sinThetaH = sqrtf(1.0f - cosThetaH * cosThetaH);
			Vec3 h(sinThetaH * cosf(phi), sinThetaH * sinf(phi), cosThetaH);
			// Reflect woLocal about h
			wi = -woLocal + h * 2.0f * Dot(woLocal, h);
			
			wi = shadingData.frame.toWorld(wi);
			// pdf
			float pdf_h = ((n + 1.0f) * powf(std::max(h.z, 0.0f), n)) / (2.0f * M_PI);
			pdfSpec = pdf_h / (4.0f * fabsf(Dot(woLocal, h)) + 1e-6f);
			// assume white specular
			reflectedColour = Colour(1.0f, 1.0f, 1.0f);
			pdf = F * pdfSpec;
		}
		else {
			// Diffuse Sampling
			Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			wi = shadingData.frame.toWorld(wiLocal);

			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
			pdfDiff = SamplingDistributions::cosineHemispherePDF(wiLocal);
			pdf = (1.0f - F) * pdfDiff;
		}
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
        Vec3 wiLocal = shadingData.frame.toLocal(wi);
        Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
        float cosTheta = fabsf(woLocal.z);
        // Compute Fresnel
        float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
        // Diffuse term
        Colour diffuse = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
        // Specular term using a Phong lobe
        float n = alphaToPhongExponent();
        
        Vec3 h = (wiLocal + woLocal).normalize();
        float specularTerm = ((n + 2.0f) / (2.0f * M_PI)) * powf(std::max(h.z, 0.0f), n);
        Colour specular = Colour(1.0f, 1.0f, 1.0f) * specularTerm;
        // return a weighted sum
        return diffuse * (1.0f - F) + specular * F;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float cosTheta = fabsf(woLocal.z);
		float F = ShadingHelper::fresnelDielectric(cosTheta, intIOR, extIOR);
		// Diffuse PDF
		float pdfDiff = SamplingDistributions::cosineHemispherePDF(wiLocal);
		// Specular PDF
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