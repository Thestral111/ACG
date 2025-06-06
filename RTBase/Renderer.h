﻿#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include "Denoiser.h"

#define MAX_DEPTH 5
class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		//std::cout << "Luminance: " << emitted.Lum() << std::endl;
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					//return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
					// Evaluate BSDF and get its PDF for the same direction.
					Colour f = shadingData.bsdf->evaluate(shadingData, wi);
					float pdf_bsdf = shadingData.bsdf->PDF(shadingData, wi);
					// MIS weight (power heuristic, exponent 2)
					float weight = (pdf * pdf) / (pdf * pdf + pdf_bsdf * pdf_bsdf + 1e-6f);
					// Compute final contribution: include the cosine term GTerm and normalize by pmf * pdf.
					return f * emitted * GTerm * weight / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		// Add pathtracer code here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		//std::cout << shadingData.t << std::endl;
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			if (depth > MAX_DEPTH)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			/*Colour bsdf;
			float pdf;
			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());*/
			//Colour bsdf;
			Colour indirect;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			//pdf = SamplingDistributions::cosineHemispherePDF(wi);
			//wi = shadingData.frame.toWorld(wi);
			//bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			r.init(shadingData.x + (wi * EPSILON), wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(r.dir);
		//return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (intersection.t < FLT_MAX) {
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}


	// --- Light Tracing Implementations ---
	// Connect a point on a surface to the camera.
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		float x, y;
		if (!scene->camera.projectOntoCamera(p, x, y)) return;

		Vec3 toCam = scene->camera.origin - p;
		if (toCam.length() <= 0.0f) return;
		float d2 = toCam.lengthSq();
		float r2Inv = 1.0f / d2;
		toCam = toCam.normalize();

		float cosThetaCam = max(Dot(-toCam, scene->camera.viewDirection), 0.0f);
		float cosThetaLight = max(Dot(toCam, n), 0.0f);
		float G = (cosThetaCam * cosThetaLight) * r2Inv;

		if (G > 0) {
			if (!scene->visible(p, scene->camera.origin)) {
				return;
			}
			else {
				//float cos2 = cosThetaCam * cosThetaCam;
				//float cos4 = cos2 * cos2;
				float cos4 = pow(cosThetaCam, 4);
				float we = 1.0f / (scene->camera.Afilm * cos4);
				
				film->splat(x, y, col * G * we);
				// Update the AOV buffers
				/*Ray camRay = scene->camera.generateRay(x + 0.5f, y + 0.5f);
				film->updateAOV(x, y, albedo(camRay), viewNormals(camRay));*/
			}
		}
		else {
			return;
		}
	}


	// Start a light path from a light source.
	void lightTrace(Sampler* sampler) {
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		//if (!light) return;

		float pdfPosition = 0.0f;
		Vec3 lightPos = light->samplePositionFromLight(sampler, pdfPosition);
		float pdfDirection = 0.0f;
		Vec3 lightDir = light->sampleDirectionFromLightLT(sampler, pdfDirection);
		//lightDir = lightDir.normalize();
		float pdfTotal = pmf * pdfPosition * pdfDirection;
		//if (pdfTotal <= 0.0f) return;
		Colour Le = light->evaluate(-lightDir);
		Colour col = Le / pdfPosition;
		ShadingData dummySD;
		dummySD.x = lightPos;
		connectToCamera(lightPos, light->normal(dummySD, -lightDir), col);
		Ray r(lightPos + (lightDir * EPSILON), lightDir);
		Le = Le * lightDir.dot(light->normal(dummySD, -lightDir));
		//pdfTotal *= pdfDirection;
		lightTracePath(r, Colour(1.0f, 1.0f, 1.0f), Le / pdfTotal, sampler);
	}

	// Recursive light tracing function.
	void lightTracePathRecursive(Ray r, Colour pathThroughput, Colour Le, Sampler* sampler, int depth) {
		
		IntersectionData isect = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(isect, r);
		if (shadingData.t >= FLT_MAX)
			return;

		Vec3 wi = scene->camera.origin - shadingData.x;

		// Connect the current hit point to the camera.
		connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le);
		if (depth > MAX_DEPTH)
			return;
		// Terminate the path probabilistically via Russian roulette.
		float rrProbability = min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() < rrProbability){
			pathThroughput = pathThroughput / rrProbability;
		}
		else
		{
			return;
		}

		// Sample a new direction from the BSDF.
		float pdf;
		Colour bsdfVal;
		Vec3 wi1 = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdf);
		//wi1 = wi1.normalize();
		/*if (pdf <= 0)
			return;*/

		pathThroughput = pathThroughput * bsdfVal * fabsf(Dot(wi1, shadingData.sNormal)) / pdf;

		// Spawn a new ray from a point offset by EPSILON to avoid self-intersections.
		r.init(shadingData.x + wi1 * EPSILON, wi1);

		// Recursively trace the new ray.
		lightTracePathRecursive(r, pathThroughput, Le, sampler, depth + 1);
	}

	

	// Entry point for recursive light tracing.
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler) {
		lightTracePathRecursive(r, pathThroughput, Le, sampler, 0);
	}
	// --- End Light Tracing Implementations ---


	struct VPL {
		Vec3 position;
		Vec3 normal;
		Colour flux; 
	};
	std::vector<VPL> precomputedVPLs;

	// --- Instant Radiosity Functions ---
	// VPL structure is defined above.
	std::vector<VPL> traceVPLs(Sampler* sampler, int numVPLs)
	{
		std::vector<VPL> vplList;
		vplList.reserve(numVPLs);
		for (int i = 0; i < numVPLs; i++) {
			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);
			float pdfPos, pdfDir;
			Vec3 lightPos = light->samplePositionFromLight(sampler, pdfPos);
			Vec3 lightDir = light->sampleDirectionFromLight(sampler, pdfDir);
			Colour Le = light->evaluate(lightDir);
			Colour throughput = Le / (pmf * pdfPos * pdfDir + 1e-6f);
			Ray r(lightPos, lightDir);
			IntersectionData isect = scene->traverse(r);
			if (isect.t < FLT_MAX) {
				ShadingData shadingData = scene->calculateShadingData(isect, r);
				if (!shadingData.bsdf->isPureSpecular()) {
					VPL vpl;
					vpl.position = shadingData.x;
					vpl.normal = shadingData.sNormal;
					vpl.flux = throughput; // / float(numVPLs)
					vplList.push_back(vpl);
				}
			}
		}
		return vplList;
	}

	// Precompute VPLs
	void precomputeVPLs(Sampler* sampler, int numVPLs) {
		precomputedVPLs = traceVPLs(sampler, numVPLs);
	}

	Colour evaluateInstantRadiosityPixel(Ray& cameraRay, const std::vector<VPL>& vplList)
	{
		IntersectionData isect = scene->traverse(cameraRay);
		if (isect.t >= FLT_MAX) {
			return scene->background->evaluate(cameraRay.dir);
		}
		ShadingData shadingData = scene->calculateShadingData(isect, cameraRay);
		Colour indirect(0.0f, 0.0f, 0.0f);
		for (const VPL& vpl : vplList) {
			Vec3 dir = vpl.position - shadingData.x;
			float d2 = dir.lengthSq();
			dir = dir.normalize();
			float cosThetaCam = max(Dot(shadingData.sNormal, dir), 0.0f);
			float cosThetaVPL = max(Dot(vpl.normal, -dir), 0.0f);
			float G = (cosThetaCam * cosThetaVPL) / (d2 + 1e-6f);
			if (!scene->visible(shadingData.x, vpl.position))
				continue;
			Colour f = shadingData.bsdf->evaluate(shadingData, dir);
			indirect = indirect + (vpl.flux * f * G);
		}
		return indirect;
	}
	// --- End Instant Radiosity Functions ---

	
	static const int TILE_SIZE = 32;
	int renderMode = 2;
	int PATH_TRACING = 1;
	int INSTANT_RADIOSITY = 2;
	// Render for path tracing and IR with tilebased rendering
	void render()
	{
		film->incrementSPP();
		int numTilesX = (film->width + TILE_SIZE - 1) / TILE_SIZE;
		int numTilesY = (film->height + TILE_SIZE - 1) / TILE_SIZE;
		int totalTiles = numTilesX * numTilesY;
		std::atomic<int> nextTile(0);
		std::vector<std::thread> workers;
		int numThreads = numProcs;
		workers.reserve(numThreads);
		precomputeVPLs(samplers, 100);
		
		// Worker function: each thread processes one tile at a time
		auto workerFunc = [=, &nextTile](int id) {
			//int threadID = std::this_thread::get_id().hash();
			int threadID = static_cast<int>(std::hash<std::thread::id>{}(std::this_thread::get_id()));
			//int threadID = std::this_thread::get_id();
			
			while (true)
			{
				int tileIndex = nextTile.fetch_add(1, std::memory_order_relaxed);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				int startX = tileX * TILE_SIZE;
				int startY = tileY * TILE_SIZE;
				int endX = min(startX + TILE_SIZE, film->width);
				int endY = min(startY + TILE_SIZE, film->height);

				//printf("Thread %d rendering tile (%d, %d) to (%d, %d)\n", id, startX, startY, endX, endY);
				for (int y = startY; y < endY; y++)
				{
					for (int x = startX; x < endX; x++)
					{
						float px = x + 0.5f;
						float py = y + 0.5f;
						Ray ray = scene->camera.generateRay(px, py);
						//Colour col = viewNormals(ray);
						//Colour col = albedo(ray);
						Colour initialThroughput(1.0f, 1.0f, 1.0f);
						Colour col;
						if (renderMode == INSTANT_RADIOSITY) {

							col = evaluateInstantRadiosityPixel(ray, precomputedVPLs);
						}
						else
						{
						
							col = pathTrace(ray, initialThroughput, 0, samplers);
						}
						
						film->splat(px, py, col);
						film->updateAOV(x, y, albedo(ray), viewNormals(ray));
						unsigned char r = static_cast<unsigned char>(col.r * 255);
						unsigned char g = static_cast<unsigned char>(col.g * 255);
						unsigned char b = static_cast<unsigned char>(col.b * 255);
						film->tonemap(x, y, r, g, b);
						canvas->draw(x, y, r, g, b);
					}
				}
			}
			};

		// Launch a fixed number of threads.
		for (int i = 0; i < numThreads; i++)
		{

			workers.emplace_back(workerFunc, i);
		}

		// Wait for all threads to complete
		for (auto& worker : workers) {
			worker.join();
		}

		Denoiser::apply(film);
		savePNG("test.png");
		saveHDR("test.hdr");
		stbi_write_png("denoised.png",film->width, film->height, 3, film->colorBuffer, canvas->getWidth() * 3);
		stbi_write_hdr("denoised.hdr", film->width, film->height, 3, film->colorBuffer);
	}

	

	void renderLT()
	{
		film->incrementSPP();
		int numTilesX = (film->width + TILE_SIZE - 1) / TILE_SIZE;
		int numTilesY = (film->height + TILE_SIZE - 1) / TILE_SIZE;
		int totalTiles = numTilesX * numTilesY;
		std::atomic<int> nextTile(0);
		std::vector<std::thread> workers;
		int numThreads = numProcs;
		workers.reserve(numThreads);

		// For each tile, we'll trace a fixed number of light paths.
		const int lightPathsPerTile = 1000; // Adjust this value as needed
		int paths = (film->width * film->height) / totalTiles;
		cout << "Paths per tile: " << paths << endl;

		auto workerFunc = [=, &nextTile](int threadID) {
			while (true) {
				int tileIndex = nextTile.fetch_add(1, std::memory_order_relaxed);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				// For this tile, trace light paths.
				for (int i = 0; i < lightPathsPerTile; i++) {
					// Call the existing light tracing method.
					lightTrace(samplers);
				}
				// After tracing, update the canvas for this tile.
				int startX = tileX * TILE_SIZE;
				int startY = tileY * TILE_SIZE;
				int endX = min(startX + TILE_SIZE, film->width);
				int endY = min(startY + TILE_SIZE, film->height);
				for (int y = startY; y < endY; y++) {
					for (int x = startX; x < endX; x++) {
						unsigned char r, g, b;
						film->tonemap(x, y, r, g, b);
						canvas->draw(x, y, r, g, b);
					}
				}
			}
			};

		for (int i = 0; i < numProcs; i++) {
			workers.emplace_back(workerFunc, i);
		}
		for (auto& worker : workers) {
			worker.join();
		}
		savePNG("test.png");
	}


	void renderAdaptive() {
		film->incrementSPP();

		// If using instant radiosity, precompute VPLs once per frame.
		if (renderMode == INSTANT_RADIOSITY) {
			precomputeVPLs(samplers, 100); // e.g., 100 VPLs; adjust as needed.
		}

		int numTilesX = (film->width + TILE_SIZE - 1) / TILE_SIZE;
		int numTilesY = (film->height + TILE_SIZE - 1) / TILE_SIZE;
		int totalTiles = numTilesX * numTilesY;
		std::atomic<int> nextTile(0);
		std::vector<std::thread> workers;
		int numThreads = numProcs;
		workers.reserve(numThreads);

		// Parameters for adaptive sampling per pixel.
		const int initialSamples = 4;       // Initial fixed samples per pixel.
		const int maxSPP = 32;              // maximum samples per pixel allowed
		const float varianceThreshold = 0.01f; // Variance threshold to trigger extra sampling.

		auto workerFunc = [=, &nextTile](int id) {
			while (true) {
				int tileIndex = nextTile.fetch_add(1, std::memory_order_relaxed);
				if (tileIndex >= totalTiles)
					break;
				int tileX = tileIndex % numTilesX;
				int tileY = tileIndex / numTilesX;
				int startX = tileX * TILE_SIZE;
				int startY = tileY * TILE_SIZE;
				int endX = min(startX + TILE_SIZE, film->width);
				int endY = min(startY + TILE_SIZE, film->height);

				// Allocate per-tile buffers for accumulating samples.
				int tileWidth = endX - startX;
				int tileHeight = endY - startY;
				std::vector<Colour> pixelSum(tileWidth * tileHeight, Colour(0.0f, 0.0f, 0.0f));
				std::vector<Colour> pixelSumSq(tileWidth * tileHeight, Colour(0.0f, 0.0f, 0.0f));
				std::vector<int> pixelSamples(tileWidth * tileHeight, 0);

				// Initial Pass: take a fixed number of samples 
				for (int ty = 0; ty < tileHeight; ty++) {
					for (int tx = 0; tx < tileWidth; tx++) {
						int idx = ty * tileWidth + tx;
						for (int s = 0; s < initialSamples; s++) {
							float px = (startX + tx) + 0.5f;
							float py = (startY + ty) + 0.5f;
							Ray ray = scene->camera.generateRay(px, py);
							Colour sampleColor;
							// Choose the rendering mode:
							if (renderMode == PATH_TRACING) {
								Colour throughput(1.0f, 1.0f, 1.0f);
								sampleColor = pathTrace(ray, throughput, 0, samplers);
							}
							else if (renderMode == INSTANT_RADIOSITY) {
								sampleColor = evaluateInstantRadiosityPixel(ray, precomputedVPLs);
							}
							pixelSum[idx] = pixelSum[idx] + sampleColor;
							pixelSumSq[idx] = pixelSumSq[idx] + (sampleColor * sampleColor);
							pixelSamples[idx]++;
						}
					}
				}

				// Compute Tile-Level Variance
				float totalVar = 0.0f;
				int numPixels = tileWidth * tileHeight;
				for (int i = 0; i < numPixels; i++) {
					Colour avg = pixelSum[i] / float(pixelSamples[i]);
					Colour varColor = (pixelSumSq[i] / float(pixelSamples[i])) - (avg * avg);
					totalVar += varColor.Lum();
				}
				float tileAvgVar = totalVar / float(numPixels);

				// Extra Sampling if Variance is High
				if (tileAvgVar >= varianceThreshold) {
					for (int ty = 0; ty < tileHeight; ty++) {
						for (int tx = 0; tx < tileWidth; tx++) {
							int idx = ty * tileWidth + tx;
							while (pixelSamples[idx] < maxSPP) {
								float px = (startX + tx) + 0.5f;
								float py = (startY + ty) + 0.5f;
								Ray ray = scene->camera.generateRay(px, py);
								Colour sampleColor;
								if (renderMode == PATH_TRACING) {
									Colour throughput(1.0f, 1.0f, 1.0f);
									sampleColor = pathTrace(ray, throughput, 0, samplers);
								}
								else if (renderMode == INSTANT_RADIOSITY) {
									sampleColor = evaluateInstantRadiosityPixel(ray, precomputedVPLs);
								}
								pixelSum[idx] = pixelSum[idx] + sampleColor;
								pixelSumSq[idx] = pixelSumSq[idx] + (sampleColor * sampleColor);
								pixelSamples[idx]++;
							}
						}
					}
				}

				// Write Final Colours
				for (int ty = 0; ty < tileHeight; ty++) {
					for (int tx = 0; tx < tileWidth; tx++) {
						int idx = ty * tileWidth + tx;
						Colour finalColor = pixelSum[idx] / float(pixelSamples[idx]);
						float px = (startX + tx) + 0.5f;
						float py = (startY + ty) + 0.5f;
						film->splat(px, py, finalColor);
						unsigned char r = static_cast<unsigned char>(finalColor.r * 255);
						unsigned char g = static_cast<unsigned char>(finalColor.g * 255);
						unsigned char b = static_cast<unsigned char>(finalColor.b * 255);
						film->tonemap(startX + tx, startY + ty, r, g, b);
						canvas->draw(startX + tx, startY + ty, r, g, b);
					}
				}

			}
			};

		for (int i = 0; i < numThreads; i++) {
			workers.emplace_back(workerFunc, i);
		}
		for (auto& worker : workers) {
			worker.join();
		}
		/*Denoiser::apply(film);*/
		savePNG("test.png");
	}

	int getSPP()
	{
		return film->SPP;
	}

	void saveHDR(std::string filename)
	{
		film->save(filename);
	}

	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};