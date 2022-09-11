#ifndef TRACE_H
#define TRACE_H

#include "slVector.H"
#include <vector>
#include <algorithm>

inline float ffmin(float a, float b) { return a < b ? a : b; }
inline float ffmax(float a, float b) { return a > b ? a : b; }

class Ray {
public:
	SlVector3 e;
	SlVector3 d;
	int depth;
	Ray(const SlVector3& _e, const SlVector3& _d, int _depth = 0) : e(_e), d(_d), depth(_depth) {};
};

class Fill {
public:
	SlVector3 color;
	double kd, ks, shine, t, ior;
};

class HitRecord {
public:
	double t, alpha, beta, gamma;
	SlVector3 p, n, v;
	Fill f;
	int raydepth;
};

class Light {
public:
	SlVector3 p, c;
};

// Axis Aligned Bounding Box
class AABB
{
public:
	AABB() {}
	AABB(const SlVector3& a, const SlVector3& b)
	{
		_min = a;
		_max = b;
	}

	SlVector3 min() const { return _min; }
	SlVector3 max() const { return _max; }

	// Intersection detection of ray and box
	bool hit(const Ray& r, float tmin, float tmax) const
	{
		for (int i = 0; i < 3; i++)
		{
			float t0 = ffmin((_min[i] - r.e[i]) / r.d[i], (_max[i] - r.e[i]) / r.d[i]);
			float t1 = ffmax((_min[i] - r.e[i]) / r.d[i], (_max[i] - r.e[i]) / r.d[i]);
			tmin = ffmax(t0, tmin);
			tmax = ffmin(t1, tmax);
			if (tmax <= tmin)
				return false;
		}
		return true;
	}

	// To get the box with two boxes
	static AABB SurroundingBox(AABB box0, AABB box1)
	{
		SlVector3 small(fmin(box0.min().x(), box1.min().x()), fmin(box0.min().y(), box1.min().y()), fmin(box0.min().z(), box1.min().z()));
		SlVector3 big(fmax(box0.max().x(), box1.max().x()), fmax(box0.max().y(), box1.max().y()), fmax(box0.max().z(), box1.max().z()));
		return AABB(small, big);
	}

private:
	SlVector3 _min; 
	SlVector3 _max; 
};


class Surface {
public:
	Fill mat;
	virtual bool intersect(const Ray& r, double t0, double t1, HitRecord& hr) const = 0;
	virtual bool boundingBox(AABB& box) const = 0;
	virtual ~Surface() {};
};

class Triangle : public Surface {
	SlVector3 a, b, c;
public:
	Fill mat;
	Triangle(const SlVector3& _a, const SlVector3& _b, const SlVector3& _c, const Fill& _f) : a(_a), b(_b), c(_c), mat(_f) {};
	virtual bool intersect(const Ray& r, double t0, double t1, HitRecord& hr) const;
	virtual bool boundingBox(AABB& box) const;
};

class TrianglePatch : public Triangle {
	SlVector3 n1, n2, n3;
public:		
	Fill mat;
	TrianglePatch(const SlVector3& _a, const SlVector3& _b, const SlVector3& _c,
		const SlVector3& _n1, const SlVector3& _n2, const SlVector3& _n3, const Fill& _f)
		: Triangle(_a, _b, _c, _f), n1(_n1), n2(_n2), n3(_n3) {};
	virtual bool intersect(const Ray& r, double t0, double t1, HitRecord& hr) const;
};

class Sphere : public Surface {
	SlVector3 c;
	double rad;	
public:
	Fill mat;
	Sphere(const SlVector3& _c, double _r, const Fill& _f) : c(_c), rad(_r), mat(_f) {};
	virtual bool intersect(const Ray& r, double t0, double t1, HitRecord& hr) const;
	virtual bool boundingBox(AABB& box) const;
};

class BVHNode : public Surface
{
public:
	BVHNode() {}
	BVHNode(std::vector<Surface* > l);
	virtual ~BVHNode(){}
	virtual bool intersect(const Ray& r, double t0, double t1, HitRecord& hr) const;
	virtual bool boundingBox(AABB& box) const;

	Surface* left;
	Surface* right;
	AABB box;
};

// Compare the coordinates of the two bounding boxes
bool box_x_compare(const Surface* a, const Surface* b);
bool box_y_compare(const Surface* a, const Surface* b);
bool box_z_compare(const Surface* a, const Surface* b);

BVHNode::BVHNode(std::vector<Surface* > l)
{
	// Sort a random axis from x,y,z
	int axis = int(3 * rand() / RAND_MAX);
	if (axis == 0)
		std::sort(l.begin(), l.end(), box_x_compare);
	else if (axis == 1)
		std::sort(l.begin(), l.end(), box_y_compare);
	else
		std::sort(l.begin(), l.end(), box_z_compare);

	// Generate a binary tree
	if (l.size() == 1)
	{ // One object
		left = right = l[0];
	}
	else if (l.size() == 2)
	{ // Two objects
		left = l[0];
		right = l[1];
	}
	else
	{ // More than two objects, recursively generate a subtree
		left = new BVHNode(std::vector<Surface* >(l.begin(), l.begin() + l.size() / 2));
		right = new BVHNode(std::vector<Surface* >(l.begin() + l.size() / 2, l.end()));
	}

	// To get the bounding box
	AABB box_left, box_right;
	if (!left->boundingBox(box_left) || !right->boundingBox(box_right))
		std::cerr << "no bounding box in bvh_node constructor\n";
	box = AABB::SurroundingBox(box_left, box_right);
}

bool BVHNode::intersect(const Ray &r, double t0, double t1, HitRecord &hr) const
{
	if (box.hit(r, t0, t1))
	{
		HitRecord left_rec, right_rec;
		bool hit_left = left->intersect(r, t0, t1, left_rec);
		bool hit_right = right->intersect(r, t0, t1, right_rec);
		if (hit_left && hit_right)
		{
			if (left_rec.t < right_rec.t)
				hr = left_rec;
			else
				hr = right_rec;
			return true;
		}
		else if (hit_left)
		{
			hr = left_rec;
			return true;
		}
		else if (hit_right)
		{
			hr = right_rec;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}

bool BVHNode::boundingBox(AABB& b) const
{
	b = box;
	return true;
}

bool box_x_compare(const Surface* a, const Surface* b) 
{
	AABB box_left, box_right;
	if (!a->boundingBox(box_left) || !b->boundingBox(box_right))
		std::cerr << "no bounding box in bvh_node constructor\n";
	return box_left.min().x() - box_right.min().x() < 0.0;
}

bool box_y_compare(const Surface* a, const Surface* b)
{
	AABB box_left, box_right;
	if (!a->boundingBox(box_left) || !b->boundingBox(box_right))
		std::cerr << "no bounding box in bvh_node constructor\n";
	return box_left.min().y() - box_right.min().y() < 0.0;
}

bool box_z_compare(const Surface* a, const Surface* b)
{
	AABB box_left, box_right;
	if (!a->boundingBox(box_left) || !b->boundingBox(box_right))
		std::cerr << "no bounding box in bvh_node constructor\n";
	return box_left.min().z() - box_right.min().z() < 0.0;
}

class Tracer {
	SlVector3 bcolor, eye, at, up;
	double angle, hither;
	unsigned int res[2];
	//std::vector<std::pair<Surface*, Fill> > surfaces;
	std::vector<Surface* > list;
	std::vector<Light> lights;
	double shadowbias;
	SlVector3* im;
public:
	Tracer(const std::string& fname);
	~Tracer();
	void traceImage();
	SlVector3 trace(const Ray& ray, double t0, double t1) const;
	SlVector3 shade(const HitRecord& hr) const;
	void writeImage(const std::string& fname);
	bool color;
	int samples;
	double aperture;
	int maxraydepth;
	BVHNode bvh_tree;
};

#endif
