#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class GaussianFilter : public ImageFilter {
public:
	float d;
	float radius;
	float alpha;
	float boundaryExp;
	// Constructor
	//   alpha controls how quickly the Gaussian falls off.
	//   radius is the maximum distance (in pixels, typically) beyond which the filter is zero.
	GaussianFilter(float alpha, float radius)
		: alpha(alpha), radius(radius)
	{
		// Precompute the exponential at the boundary once
		boundaryExp = std::exp(-alpha * radius * radius);

		// (Optional) Precompute a normalization factor if you want the filter to peak at 1.0
		// norm = 1.0f / (1.0f - boundaryExp);
	}

	float filter(float x, float y) const {
		float d2 = x * x + y * y;
		// If distance is beyond the filter's radius, return zero (finite support).
		if (d2 > radius * radius)
			return 0.0f;
		// Difference of exponentials: e^(-alpha*d^2) - e^(-alpha*radius^2).
		// At d = radius, this equals 0. At d = 0, it equals 1 - boundaryExp.
		float val = std::exp(-alpha * d2) - boundaryExp;
		return val;
	}

	int size() const
	{
		return 0;
	}
};

class MitchellFilter : public ImageFilter
{
public:
	float B = 1 / 3;
	float C = 1 / 3;

	// The 1D Mitchell-Netravali kernel.
	inline float mitchell1D(float x) const
	{
		// Work with the absolute value.
		x = fabsf(x);

		if (x < 1.0f)
		{
			// For |x| < 1.
			return (1.0f / 6.0f) * ((12 - 9 * B - 6 * C) * x * x * x +
				(-18 + 12 * B + 6 * C) * x * x +
				(6 - 2 * B));
		}
		else if (x < 2.0f)
		{
			// For 1 ≤ |x| < 2.
			return (1.0f / 6.0f) * ((-B - 6 * C) * x * x * x +
				(6 * B + 30 * C) * x * x +
				(-12 * B - 48 * C) * x +
				(8 * B + 24 * C));
		}
		else
		{
			// Outside the support.
			return 0.0f;
		}
	}

	float filter(const float x, const float y) const override
	{
		return mitchell1D(x) * mitchell1D(y);
	}

};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	float* colorBuffer;       
	float* albedoBuffer;     
	float* normalBuffer;      
	int SPP;
	ImageFilter* filter;
	void splat(const float x, const float y, const Colour& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25]; // Storage to cache weights 
		unsigned int indices[25]; // Store indices to minimize computations 
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(j, i);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			int idx = indices[i];
			film[idx] = film[idx] + (L * filterWeights[i] / total);
			// Update the colorBuffer with the new colour
			colorBuffer[idx * 3 + 0] = film[idx].r;
			colorBuffer[idx * 3 + 1] = film[idx].g;
			colorBuffer[idx * 3 + 2] = film[idx].b;
		}
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		// Return a tonemapped pixel at coordinates x, y.
		Colour pixel = film[y * width + x] * exposure / (float)SPP;
		r = std::min(powf(std::max(pixel.r, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		g = std::min(powf(std::max(pixel.g, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
		b = std::min(powf(std::max(pixel.b, 0.0f), 1.0f / 2.2f) * 255, 255.0f);
	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		colorBuffer = new float[width * height * 3];
		albedoBuffer = new float[width * height * 3];
		normalBuffer = new float[width * height * 3];
		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		memset(colorBuffer, 0, width * height * 3 * sizeof(float));
		memset(albedoBuffer, 0, width * height * 3 * sizeof(float));
		memset(normalBuffer, 0, width * height * 3 * sizeof(float));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}

	void updateAOV(int x, int y, const Colour& albedo, const Colour& normal) {
		int idx = y * width + x;
		albedoBuffer[idx * 3 + 0] = albedo.r;
		albedoBuffer[idx * 3 + 1] = albedo.g;
		albedoBuffer[idx * 3 + 2] = albedo.b;
		normalBuffer[idx * 3 + 0] = normal.r;
		normalBuffer[idx * 3 + 1] = normal.g;
		normalBuffer[idx * 3 + 2] = normal.b;
	}

};