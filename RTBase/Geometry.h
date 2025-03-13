#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		float denominator = n.dot(r.dir); // n ?d (dot product of normal and ray direction)

		// Check if ray is parallel to the plane
		if (std::fabs(denominator) < 1e-6)  // Avoid division by zero
		{
			return false;  // No intersection
		}

		float numerator = d - n.dot(r.o); // -(n ?o - d)
		t = numerator / denominator;  // Solve for t

		return (t >= 0);  // Intersection is valid only if t is non-negative
		return false;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p; // original
		e2 = vertices[0].p - vertices[2].p;
		//e1 = vertices[1].p - vertices[0].p;
		//e2 = vertices[2].p - vertices[0].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	
	/*
	// Ray-Plane + Barycentric Test
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		

		// checks if the ray intersects the plane
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		// Compute intersection point
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }

		return true;
	}
	*/
	
	// Moller-Trumbore intersection algorithm
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const {
		// Recover conventional edges using the stored ones.
		Vec3 v0 = vertices[0].p;
		Vec3 E1 = -e2 - e1;  // = v1 - v0
		Vec3 E2 = -e2;       // = v2 - v0
		// Compute determinant
		Vec3 h = r.dir.cross(E2);
		float det = E1.dot(h);
		// If determinant is near zero, the ray lies in the plane of the triangle
		if (std::fabs(det) < EPSILON)
			return false;

		Vec3 s = r.o - vertices[0].p;

		// Compute u parameter and test bounds
		float invDet = 1.0f / det;
		u = s.dot(h) * invDet;
		if (u < 0.0f || u > 1.0f)
			return false;

		// Compute v parameter and test bounds
		Vec3 q = s.cross(E1);
		v = r.dir.dot(q) * invDet;
		if (v < 0.0f || (u + v) > 1.0f)
			return false;

		// Compute t to find intersection point
		t = E2.dot(q) * invDet;

		return t > EPSILON; // The intersection is valid only if t is positive

	}
	

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		float r1 = sampler->next();
		float r2 = sampler->next();
		// Compute barycentric coordinates via the "square root" parameterization.
		float sqrt1 = std::sqrt(r1);
		float alpha = 1.0f - sqrt1;
		float beta = r2 * sqrt1;
		float gamma = 1.0f - alpha - beta;
		// Convert barycentric coords into a position on the triangle.
		// P = alpha * v0 + beta * v1 + gamma * v2
		Vec3 p = vertices[0].p * alpha + vertices[1].p * beta + vertices[2].p * gamma;	

		// For a uniform sample on the triangle, pdf = 1 / area.
		pdf = 1.0f / area;

		return p;
		//return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		// Compute intersection distances for the x-axis.
		float tx1 = (min.x - r.o.x) * r.invDir.x;
		float tx2 = (max.x - r.o.x) * r.invDir.x;
		float tmin_x = std::min(tx1, tx2);
		float tmax_x = std::max(tx1, tx2);

		// Compute intersection distances for the y-axis.
		float ty1 = (min.y - r.o.y) * r.invDir.y;
		float ty2 = (max.y - r.o.y) * r.invDir.y;
		float tmin_y = std::min(ty1, ty2);
		float tmax_y = std::max(ty1, ty2);

		// Compute intersection distances for the z-axis.
		float tz1 = (min.z - r.o.z) * r.invDir.z;
		float tz2 = (max.z - r.o.z) * r.invDir.z;
		float tmin_z = std::min(tz1, tz2);
		float tmax_z = std::max(tz1, tz2);

		// The entry point is the largest tmin, and the exit is the smallest tmax.
		float tmin_final = std::max(tmin_x, std::max(tmin_y, tmin_z));
		float tmax_final = std::min(tmax_x, std::min(tmax_y, tmax_z));

		// If the exit is behind the ray or there is no overlap, there's no intersection.
		if (tmax_final < 0 || tmin_final > tmax_final)
			return false;

		// Return the entry intersection distance.
		t = tmin_final;
		return true;
	}
	// Simple intersection test that ignores the entry distance.
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}
	// Computes the surface area of the bounding box.
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
