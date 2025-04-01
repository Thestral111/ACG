#pragma once

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
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
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

	// Helper: Try to connect a point p on a light path to the camera.
	// This function returns the contribution if p projects onto the camera; otherwise, it returns black.
	Colour connectToCamera(const Vec3& p, const Vec3& n, const Colour& pathThroughput) {
		float pixelX, pixelY;
		if (scene->camera.projectOntoCamera(p, pixelX, pixelY)) {
			// Compute the direction from the point to the camera.
			Vec3 toCamera = (scene->camera.origin - p);
			float distanceSquared = toCamera.lengthSq();
			toCamera = toCamera.normalize();
			// Compute the cosine between this direction and the camera's normal.
			float cosThetaCam = max(Dot(toCamera, scene->camera.viewDirection), 0.0f);
			// Geometry term: this is a simplified form.
			float G = cosThetaCam / (distanceSquared + 1e-6f);
			return pathThroughput * G;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	// Recursive light tracing function.
	// It starts at a light source and then at each vertex connects to the camera and continues the light path.
	Colour lightTracePath(Ray& r, Colour pathThroughput, int depth, Sampler* sampler) {
		// Terminate if max depth reached.
		if (depth > MAX_DEPTH) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		Colour accumulated(0.0f, 0.0f, 0.0f);

		if (shadingData.t < FLT_MAX) {
			// p is the current light-path vertex.
			Vec3 p = shadingData.x;
			// Connect this vertex to the camera.
			Colour connection = connectToCamera(p, shadingData.sNormal, pathThroughput);
			accumulated = accumulated + connection;

			// Sample the BSDF at this vertex to continue the light path.
			Colour bsdfVal;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdf);
			if (pdf <= 0.0f) {
				return accumulated;
			}
			// Update the throughput. Include the cosine term (dot between the new direction and the surface normal).
			float cosTerm = fabsf(Dot(wi, shadingData.sNormal));
			Colour newThroughput = pathThroughput * bsdfVal * cosTerm / pdf;
			// Russian Roulette termination:
			float rrProb = min(newThroughput.Lum(), 0.9f);
			if (sampler->next() >= rrProb) {
				return accumulated;
			}
			newThroughput = newThroughput / rrProb;
			// Create the new ray from a point offset by EPSILON to avoid self-intersection.
			r.init(shadingData.x + wi * EPSILON, wi);
			// Continue tracing the light path.
			accumulated = accumulated + lightTracePath(r, newThroughput, depth + 1, sampler);
			return accumulated;
		}
		else {
			// If the ray does not hit anything, optionally add the background contribution.
			return pathThroughput * scene->background->evaluate(r.dir);
		}
	}

	// Wrapper function to initiate light tracing.
	Colour lightTrace(Sampler* sampler) {
		// Sample a light source.
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);

		// Sample both a position and a direction from the light.
		float pdfPos, pdfDir;
		Vec3 lightPos = light->samplePositionFromLight(sampler, pdfPos);
		Vec3 lightDir = light->sampleDirectionFromLight(sampler, pdfDir);
		// Evaluate the light emission for the given direction.
		Colour Le = light->evaluate(lightDir);
		// Create an initial ray from the light.
		Ray r(lightPos, lightDir);
		// Compute the initial throughput. The probability of this light sample is pmf * pdfPos * pdfDir.
		Colour throughput = Le / (pmf * pdfPos * pdfDir + 1e-6f);
		//std::cout << throughput.r << " " << throughput.g << " " << throughput.b << std::endl;
		// Trace the light path.
		return lightTracePath(r, throughput, 0, sampler);
	}

	static const int TILE_SIZE = 32;
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
		/*
		auto renderTile = [&](int tileX, int tileY, int threadID) {
			int startX = tileX * TILE_SIZE;
			int startY = tileY * TILE_SIZE;
			int endX = min(startX + TILE_SIZE, film->width);
			int endY = min(startY + TILE_SIZE, film->height);
			printf("Thread %d rendering tile (%d, %d) to (%d, %d)\n", threadID, startX, startY, endX, endY);

			for (int y = startY; y < endY; y++)
			{
				for (int x = startX; x < endX; x++)
				{
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);
					Colour col = viewNormals(ray);
					//Colour col = albedo(ray);
					//Colour initialThroughput(1.0f, 1.0f, 1.0f);
					//Colour col = pathTrace(ray, initialThroughput, 0, &samplers[threadID]);
					film->splat(px, py, col);
					unsigned char r = (unsigned char)(col.r * 255);
					unsigned char g = (unsigned char)(col.g * 255);
					unsigned char b = (unsigned char)(col.b * 255);
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		};
		//auto workerFunc = [&](int threadId) {
			for (int tileY = 0; tileY < numTilesY; tileY++)
			{
				for (int tileX = 0; tileX < numTilesX; tileX++)
				{
					workers.emplace_back(renderTile, tileX, tileY, workers.size() % numThreads);
					//renderTile(tileX, tileY, threadId);
				}
			}
		//};

			for (int i = 0; i < numThreads; i++)
		{
			workers.emplace_back(workerFunc, i);
		}
		*/
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
						Colour col = pathTrace(ray, initialThroughput, 0, samplers);
						//Colour col = lightTrace(samplers);
						film->splat(px, py, col);
						film->splat(px, py, col);
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