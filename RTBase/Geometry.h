#pragma once

#include "Core.h"
#include "Sampling.h"
#include <iostream>
using namespace std;

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

#define EPSILON 0.0001f // 0.001f

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
		if (std::fabs(det) < 1e-10)
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

		//cout << "t = " << t << endl;
		return t > 0.00000001f; // The intersection is valid only if t is positive

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
	void extend(const AABB& other) {
		extend(other.min);
		extend(other.max);
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

	bool rayAABB(const Ray& r, float& tMin, float& tMax)
	{
		// Compute intersection distances along x-axis.
		float tx1 = (min.x - r.o.x) * r.invDir.x;
		float tx2 = (max.x - r.o.x) * r.invDir.x;
		float tmin_x = std::min(tx1, tx2);
		float tmax_x = std::max(tx1, tx2);

		// Compute intersection distances along y-axis.
		float ty1 = (min.y - r.o.y) * r.invDir.y;
		float ty2 = (max.y - r.o.y) * r.invDir.y;
		float tmin_y = std::min(ty1, ty2);
		float tmax_y = std::max(ty1, ty2);

		// Compute intersection distances along z-axis.
		float tz1 = (min.z - r.o.z) * r.invDir.z;
		float tz2 = (max.z - r.o.z) * r.invDir.z;
		float tmin_z = std::min(tz1, tz2);
		float tmax_z = std::max(tz1, tz2);

		// The entry distance is the largest tmin of the three axes.
		tMin = std::max(tmin_x, std::max(tmin_y, tmin_z));
		// The exit distance is the smallest tmax of the three axes.
		tMax = std::min(tmax_x, std::min(tmax_y, tmax_z));

		// If tMax is behind the ray or there is no intersection interval, return false.
		if (tMax < 0 || tMin > tMax)
			return false;

		return true;
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
#define TRIANGLE_COST 2.0f // 2.0f
#define BUILD_BINS 32 // 32

// Helper function to get a component from a Vec3 by axis index.
inline float getCoord(const Vec3& v, int axis)
{
	if (axis == 0) return v.x;
	else if (axis == 1) return v.y;
	else return v.z;
}


class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset = 0;
	unsigned char num = 0;

	int startIndex;
	int endIndex;

	BVHNode()
	{
		r = NULL;
		l = NULL;
		startIndex = 0;
		endIndex = 0;
	}

	static AABB triangleBounds(const Triangle& tri)
	{
		AABB box;
		box.reset();
		box.extend(tri.vertices[0].p);
		box.extend(tri.vertices[1].p);
		box.extend(tri.vertices[2].p);
		return box;
	}

	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
		buildRecursive(inputTriangles, 0, inputTriangles.size());
	}
	
	
	void buildRecursive(std::vector<Triangle>& triangles, int start, int end)
	{
		// Compute the bounding box for all triangles in [start, end).
		bounds.reset();
		for (int i = start; i < end; i++) {
			bounds.extend(triangles[i].vertices[0].p);
			bounds.extend(triangles[i].vertices[1].p);
			bounds.extend(triangles[i].vertices[2].p);
		}

		int numTriangles = end - start;
		// If the number of triangles is below the threshold, this node becomes a leaf.
		if (numTriangles <= MAXNODE_TRIANGLES) {
			startIndex = start;
			endIndex = end;
			return;
		}

		// Compute the total surface area of this node.
		float totalArea = bounds.area();

		// Variables to track the best split overall.
		float bestCost = FLT_MAX;
		int bestAxis = -1;
		int bestSplit = -1;


		// Try each axis (0 = x, 1 = y, 2 = z).
		for (int axis = 0; axis < 3; axis++) {
			// Sort triangles by their centroid along the current axis.
			std::sort(triangles.begin() + start, triangles.begin() + end,
				[axis](const Triangle& a, const Triangle& b) {
					float ca = (axis == 0) ? a.centre().x : (axis == 1) ? a.centre().y : a.centre().z;
					float cb = (axis == 0) ? b.centre().x : (axis == 1) ? b.centre().y : b.centre().z;
					return ca < cb;
				});

			int n = numTriangles;
			// Prepare prefix and suffix arrays to store cumulative bounding boxes.
			std::vector<AABB> leftBounds(n);
			std::vector<AABB> rightBounds(n);

			// For convenience, assume you have a helper function triangleBounds(tri) that returns the AABB for a triangle.
			leftBounds[0] = triangleBounds(triangles[start]);
			for (int i = 1; i < n; i++) {
				leftBounds[i] = leftBounds[i - 1];
				leftBounds[i].extend(triangleBounds(triangles[start + i]));
			}
			rightBounds[n - 1] = triangleBounds(triangles[end - 1]);
			for (int i = n - 2; i >= 0; i--) {
				rightBounds[i] = rightBounds[i + 1];
				rightBounds[i].extend(triangleBounds(triangles[start + i]));
			}

			// Evaluate SAH cost for every possible split candidate along this axis.
			for (int i = 1; i < n; i++) {
				float leftArea = leftBounds[i - 1].area();
				float rightArea = rightBounds[i].area();
				int leftCount = i;
				int rightCount = n - i;
				// SAH cost: traversal cost (assumed 1.0f) plus weighted areas.
				float cost = 1.0f + (leftArea * leftCount + rightArea * rightCount) / totalArea;
				if (cost < bestCost) {
					bestCost = cost;
					bestAxis = axis;
					bestSplit = i;
				}
			}
		}

		// Compare the best cost to the cost of making a leaf.
		float leafCost = static_cast<float>(numTriangles) * TRIANGLE_COST;
		if (bestCost >= leafCost || bestAxis < 0) { //if (bestCost >= leafCost || bestAxis < 0)
			// Splitting is not beneficial; mark this node as a leaf.
			startIndex = start;
			endIndex = end;
			return;
		}

		/*if (bestCost >= leafCost || bestAxis < 0 ||
			bestSplit <= 0.1f * numTriangles || bestSplit >= 0.9f * numTriangles)
		{
			int mid = start + numTriangles / 2;
			startIndex = start;
			endIndex = end;
			l = new BVHNode();
			r = new BVHNode();
			l->buildRecursive(triangles, start, mid);
			r->buildRecursive(triangles, mid, end);
			return;
		}*/

		// Re-sort the triangles along the best axis using the same lambda.
		std::sort(triangles.begin() + start, triangles.begin() + end,
			[bestAxis](const Triangle& a, const Triangle& b) {
				float ca = (bestAxis == 0) ? a.centre().x : (bestAxis == 1) ? a.centre().y : a.centre().z;
				float cb = (bestAxis == 0) ? b.centre().x : (bestAxis == 1) ? b.centre().y : b.centre().z;
				return ca < cb;
			});
		// Partition the triangles at the best split.
		int mid = start + bestSplit;
		l = new BVHNode();
		r = new BVHNode();
		l->buildRecursive(triangles, start, mid);
		r->buildRecursive(triangles, mid, end);
	}
	/*
	void buildRecursive(std::vector<Triangle>& triangles, int start, int end)
	{
		// Compute overall bounding box for triangles in [start, end)
		bounds.reset();
		for (int i = start; i < end; i++) {
			bounds.extend(triangles[i].vertices[0].p);
			bounds.extend(triangles[i].vertices[1].p);
			bounds.extend(triangles[i].vertices[2].p);
		}

		int numTriangles = end - start;
		// If few triangles remain, mark this node as a leaf.
		if (numTriangles <= MAXNODE_TRIANGLES) {
			startIndex = start;
			endIndex = end;
			l = nullptr;
			r = nullptr;
			return;
		}

		// Compute centroid bounds to determine splitting axis.
		AABB centroidBounds;
		centroidBounds.reset();
		for (int i = start; i < end; i++) {
			Vec3 centroid = triangles[i].centre();
			centroidBounds.extend(centroid);
		}
		Vec3 extent = centroidBounds.max - centroidBounds.min;
		int axis = 0;
		if (extent.y > extent.x && extent.y >= extent.z)
			axis = 1;
		else if (extent.z > extent.x && extent.z > extent.y)
			axis = 2;

		// If the extent along the chosen axis is too small, mark as leaf.
		if (getCoord(extent, axis) < EPSILON) {
			startIndex = start;
			endIndex = end;
			l = nullptr;
			r = nullptr;
			return;
		}

		// --- Version B Logic: Sort triangles along the chosen axis ---
		std::sort(triangles.begin() + start, triangles.begin() + end,
			[axis](const Triangle& a, const Triangle& b) {
				float ca = (axis == 0) ? a.centre().x : (axis == 1) ? a.centre().y : a.centre().z;
				float cb = (axis == 0) ? b.centre().x : (axis == 1) ? b.centre().y : b.centre().z;
				return ca < cb;
			});

		// Build prefix and suffix arrays of bounding boxes.
		int n = numTriangles;
		std::vector<AABB> leftBounds(n);
		std::vector<AABB> rightBounds(n);
		leftBounds[0] = triangleBounds(triangles[start]);
		for (int i = 1; i < n; i++) {
			leftBounds[i] = leftBounds[i - 1];
			leftBounds[i].extend(triangleBounds(triangles[start + i]));
		}
		rightBounds[n - 1] = triangleBounds(triangles[end - 1]);
		for (int i = n - 2; i >= 0; i--) {
			rightBounds[i] = rightBounds[i + 1];
			rightBounds[i].extend(triangleBounds(triangles[start + i]));
		}

		// Evaluate SAH cost for every candidate split between 1 and n-1.
		float totalArea = bounds.area();
		float bestCost = FLT_MAX;
		int bestSplit = -1;
		for (int i = 1; i < n; i++) {
			float leftArea = leftBounds[i - 1].area();
			float rightArea = rightBounds[i].area();
			int leftCount = i;
			int rightCount = n - i;
			float cost = 1.0f + (leftArea * leftCount + rightArea * rightCount) / totalArea;
			if (cost < bestCost) {
				bestCost = cost;
				bestSplit = i;
			}
		}

		// Compute cost of a leaf (no split).
		float leafCost = static_cast<float>(numTriangles) * TRIANGLE_COST;
		if (bestCost >= leafCost) {
			// Splitting is not beneficial; mark this node as a leaf.
			startIndex = start;
			endIndex = end;
			l = nullptr;
			r = nullptr;
			return;
		}

		// Partition the triangles at the best split.
		int mid = start + bestSplit;
		l = new BVHNode();
		r = new BVHNode();
		l->buildRecursive(triangles, start, mid);
		r->buildRecursive(triangles, mid, end);
	}
	*/

	

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		float tHit;
		if (!bounds.rayAABB(ray, tHit))
			return; // Ray misses this node.

		// If leaf, test each triangle.
		if (l == nullptr && r == nullptr) // if (l == nullptr && r == nullptr)
		{
			//cout << "intersect" << endl;
			for (int i = startIndex; i < endIndex; i++) // for (unsigned int i = offset; i < offset + num; i++)
			{
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v))
				{
					//cout << "intersect" << endl;
					if (t < intersection.t)
					{
						intersection.t = t;
						intersection.ID = i;
						intersection.alpha = 1.0f - u - v;
						intersection.beta = u;
						intersection.gamma = v;
					}
				}
			}
			return;
		}

		// Otherwise, traverse children.
		if (l) l->traverse(ray, triangles, intersection);
		if (r) r->traverse(ray, triangles, intersection);
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
		float tHit;
		if (!bounds.rayAABB(ray, tHit))
			return true;

		if (l == nullptr && r == nullptr)
		{
			for (int i = startIndex; i < endIndex; i++) // for (unsigned int i = offset; i < offset + num; i++)
			{
				float t, u, v;
				if (triangles[i].rayIntersect(ray, t, u, v) && t < maxT)
					return false;
			}
			return true;
		}

		/*if (l && l->traverseVisible(ray, triangles, maxT))
			return false;
		if (r && r->traverseVisible(ray, triangles, maxT))
			return false;
		return true;*/
		//return true;
		bool leftVisible = (l ? l->traverseVisible(ray, triangles, maxT) : true);
		bool rightVisible = (r ? r->traverseVisible(ray, triangles, maxT) : true);
		return leftVisible && rightVisible;
	}
};


//class BVHNode
//{
//public:
//	AABB bounds;
//	BVHNode* l;
//	BVHNode* r;
//
//	int startIndex;
//	int endIndex;
//
//	BVHNode()
//		: l(nullptr)
//		, r(nullptr)
//		, startIndex(0)
//		, endIndex(0)
//	{
//	}
//
//	static AABB triangleBounds(const Triangle& tri)
//	{
//		AABB box;
//		box.reset();
//		box.extend(tri.vertices[0].p);
//		box.extend(tri.vertices[1].p);
//		box.extend(tri.vertices[2].p);
//		return box;
//	}
//
//	void buildRecursive(std::vector<Triangle>& triangles, int start, int end)
//	{
//		bounds.reset();
//		for (int i = start; i < end; i++) {
//			bounds.extend(triangles[i].vertices[0].p);
//			bounds.extend(triangles[i].vertices[1].p);
//			bounds.extend(triangles[i].vertices[2].p);
//		}
//
//		int numTriangles = end - start;
//		if (numTriangles <= MAXNODE_TRIANGLES)
//		{
//			this->startIndex = start;
//			this->endIndex = end;
//			return;
//		}
//
//
//		Vec3 size = bounds.max - bounds.min;
//		int axis = 0;
//		if (size.y > size.x && size.y > size.z)
//			axis = 1;
//		else if (size.z > size.x && size.z > size.y)
//			axis = 2;
//
//		std::sort(triangles.begin() + start, triangles.begin() + end,
//			[axis](const Triangle& a, const Triangle& b) {
//				float ca = (axis == 0) ? a.centre().x : (axis == 1) ? a.centre().y : a.centre().z;
//				float cb = (axis == 0) ? b.centre().x : (axis == 1) ? b.centre().y : b.centre().z;
//				return ca < cb;
//			});
//
//		int n = numTriangles;
//		std::vector<AABB> leftBounds(n);
//		std::vector<AABB> rightBounds(n);
//		leftBounds[0] = triangleBounds(triangles[start]);
//		for (int i = 1; i < n; i++) {
//			leftBounds[i] = leftBounds[i - 1];
//			leftBounds[i].extend(triangleBounds(triangles[start + i]));
//		}
//		rightBounds[n - 1] = triangleBounds(triangles[end - 1]);
//		for (int i = n - 2; i >= 0; i--) {
//			rightBounds[i] = rightBounds[i + 1];
//			rightBounds[i].extend(triangleBounds(triangles[start + i]));
//		}
//
//		float totalArea = bounds.area();
//		float bestCost = FLT_MAX;
//		int bestSplit = -1;
//
//		for (int i = 1; i < n; i++) {
//			float leftArea = leftBounds[i - 1].area();
//			float rightArea = rightBounds[i].area();
//			int leftCount = i;
//			int rightCount = n - i;
//			float cost = 1.0f + (leftArea * leftCount + rightArea * rightCount) / totalArea;
//			if (cost < bestCost) {
//				bestCost = cost;
//				bestSplit = i;
//			}
//		}
//
//		if (bestCost >= static_cast<float>(numTriangles)) {
//			this->startIndex = start;
//			this->endIndex = end;
//			return;
//		}
//
//		int mid = start + bestSplit;
//		l = new BVHNode();
//		r = new BVHNode();
//		l->buildRecursive(triangles, start, mid);
//		r->buildRecursive(triangles, mid, end);
//	}
//
//	void build(std::vector<Triangle>& triangles)
//	{
//		//triangles = inputTriangles;
//		buildRecursive(triangles, 0, static_cast<int>(triangles.size()));
//	}
//
//	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
//	{
//		float tBox;
//		if (!bounds.rayAABB(ray, tBox)) {
//			return;
//		}
//
//		if (!l && !r)
//		{
//			for (int i = startIndex; i < endIndex; i++)
//			{
//				float t, u, v;
//				if (triangles[i].rayIntersect(ray, t, u, v) && t > 1e-4f && t < intersection.t)
//				{
//					intersection.t = t;
//					intersection.alpha = 1 - u - v;
//					intersection.beta = u;
//					intersection.gamma = v;
//					intersection.ID = i;
//				}
//			}
//			return;
//		}
//
//		if (l) l->traverse(ray, triangles, intersection);
//		if (r) r->traverse(ray, triangles, intersection);
//	}
//
//	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
//	{
//		IntersectionData intersection;
//		intersection.t = FLT_MAX;
//		traverse(ray, triangles, intersection);
//		return intersection;
//	}
//
//	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, float maxT)
//	{
//		float tBox;
//		float tMin, tMax;
//		float tHit;
//		if (!bounds.rayAABB(ray, tMin, tMax)) {
//			return true;
//		}
//
//		if (tMin > maxT) {
//			return true;
//		}
//
//		if (!l && !r)
//		{
//			for (int i = startIndex; i < endIndex; i++)
//			{
//				float t, u, v;
//				if (triangles[i].rayIntersect(ray, t, u, v) && t > 1e-4f && t < maxT)
//				{
//					return false;
//				}
//			}
//			return true;
//		}
//
//		bool leftVis = l ? l->traverseVisible(ray, triangles, maxT) : true;
//		bool rightVis = r ? r->traverseVisible(ray, triangles, maxT) : true;
//		return leftVis && rightVis;
//	}
//
//};
