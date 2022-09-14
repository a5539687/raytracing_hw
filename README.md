# Assignment01_extra features

## An Anisotropic Phong BRDF Model

- The GUI is implemented by Immediate Mode GUI(IMGUI).

![image-20220911204319425](https://user-images.githubusercontent.com/82855166/189536186-8108482d-5331-4a5e-aac3-23e49d9033f6.png)
![image-20220911203744121](https://user-images.githubusercontent.com/82855166/189536264-b53f71b9-51b0-44fc-9668-10f3756dd60a.png)
![image-20220911200052488](https://user-images.githubusercontent.com/82855166/189536311-1d38d5e1-6ca8-40cf-9948-d2944e62110b.png)

# Assignment02_ray traing

## Step 1 Ray-triangle test

The main component of ray tracing, determines whether the ray intersects the spheres and triangles, finds the closest point in the pixel that intersects after traversing, and updates the data saved in HitRecord hr for the next step.It is worth mentioning that the Moller-Trumbore ray-triangle intersection algorithm is used in the function to determine whether the ray intersects triangles, which can get the desired data faster.

## Step 2 do shading

After the intersection judgment is passed, the object is shaded using the data in HitRecord hr, this part implements a simple Phong reflection model.

```text
Ambient = La * Ka;
Diffuse = Ld * Kd * max( dot(s, n), 0.0 );
Specular = Ls * Ks * pow( max( dot(r, v), 0.0 ), f );

La Ambient light intensity
Ka Ambient reflectivity
Ld Diffuse light intensity
Kd Diffuse reflectivity
s Direction from the surface point to the light source
n Normal vector at the surface point
Ls Specular light intensity
Ks Specular reflectivity
r the vector of perfect reflection
v the vector towards the viewer
f specular highlights
```

![image-20220911094912949](https://user-images.githubusercontent.com/82855166/189536330-6a03e2d4-182a-4ba8-b2d3-45edb905af07.png)

## Step 3 Check for shadows

Before calculating the normal illumination, the vector between the intersection point and the light source is checked to see if it intersects the object in the scene, and the distance between it and the light source is checked. The intersect function is passed a HitRecord dummy, and the initial value of t0 is set to be greater than 0 to prevent the computed vector between the intersection and the light source from intersecting itself, while the initial value of t1 is set to a sufficiently large number.

![shadows](https://user-images.githubusercontent.com/82855166/189536348-df49c322-90c9-441f-b32e-63669898b36a.jpg)

## Step 4 Computing reflection color

Reflection shows the wonders of ray tracing! We assume that the light bounces multiple times across the surface of an object in the scene, and when calculating the color of a point, we simply add up the illumination contribution from each bounce.

To control the number of times the light bounces, add a check on whether hr.rayDepth is less than maxraydepth.When a new reflection is obtained, hr.raydepth+1,and when hr.raydepth exceeds maxraydepth, the ray will no longer be ejected.

![reflection](https://user-images.githubusercontent.com/82855166/189536370-f00190cf-e04c-440b-9d1a-4586f71c6503.jpg)

## Step 5 Computing refraction color

I am working on the computing refraction color part, now I have a Fresnel equation to calculate the reflectance and transmittance, a function to get the refraction ray, but the obtained data still needs to be verified, I tried to add the relevant logic to the code to test.

There are some wrong images here, the reason may be a logical error, the wrong cosθ is calculated without the normalized vector, the incoming light in the opposite direction is passed in, I will continue to fix and update this in the future.

![image](https://user-images.githubusercontent.com/82855166/190091068-adf50ee5-3cc8-426c-b033-85c27e992e74.png)

### Update

I implemented a function for computing refraction vectors based on Snell's law formula, **it should be noted that when the light is refracted from the inside of the object into the air, we need to exchange the refractive index and reverse the normal**.

My refraction function is a bool function that returns **false** when total reflection occurs. I added the equation for calculating the reflectance, the Fresnel equation and one of its approximations, the Schlick equation.From the results, the resulting pictures perform basically the same.

![image-20220914112931223](https://user-images.githubusercontent.com/82855166/190090164-c146a26d-0bab-4719-8a83-6b74cd08cde8.png)

```c++
bool refract(const SlVector3& I, const SlVector3& N, const double& ior, SlVector3& r, double& k)
{
	double cos_i = dot(I, N);
	double etai = 1, etat = ior;
	SlVector3 n = N;
	if (cos_i < 0)
	{
		cos_i = -cos_i;
		std::swap(etai, etat);
		n = -N;
	}
	double eta = etai / etat;
	double sin2_i = 1.0 - sqr(cos_i);
	double sin2_t = sqr(ior) * sin2_i;
	double cos2_t = 1 - sin2_t;
	if (cos2_t < 0) return false;

	double cos_t = sqrt(cos2_t);
	r = I * eta + n * (eta * cos_i - cos_t);
	//fresnel
	float r1 = ((etai * cos_i) - (etat * cos_t)) / ((etai * cos_i) + (etat * cos_t));
	float r2 = ((etat * cos_i) - (etai * cos_t)) / ((etat * cos_i) + (etai * cos_t));
	k = (sqr(r1) + sqr(r2)) * 0.5;
	//schlick
	//float R0 = sqr((etat - etai) / (etat + etai));
	//float x = 1.0 - cos_t;
	//float x5 = pow(x, 5.0);
	//k = R0 + (1.0 - R0) * x5;
	return true;
}
```

## Extra features

- ### BVH（Bounding Volume Hierarchy）

Ray tracing generates images slowly. The slowness is due to the large amount of computation: many objects, large sampling, and recursion.There are N objects, so the complexity is O(N).I referenced "Ray Tracing: The Next Week" to build a speedup structure based on dividing the objects.

The AABB class associated with the Axis Aligned Bounding Box is added to trace.h, and a bounding box function for spheres and triangles is added to trace.cpp.

```c++
// Axis Aligned Bounding Box
class AABB
{
public:
	AABB() {}
	AABB(const SlVector3& a, const SlVector3& b){_min = a;_max = b;}
	SlVector3 min() const { return _min; }
	SlVector3 max() const { return _max; }
    // Intersection detection of ray and box
	bool hit(const Ray& r, float tmin, float tmax) const{...}
	// To get the box with two boxes
	static AABB SurroundingBox(AABB box0, AABB box1){...}
private:
	SlVector3 _min; 
	SlVector3 _max; 
};

// Bounding box function for spheres and triangles
bool Triangle::boundingBox(AABB& box) const{...}
bool Sphere::boundingBox(AABB& box) const{...}
```

The surface class and subclasses have made some changes.

```c++
class Surface {
public:
	Fill mat;
    virtual bool boundingBox(AABB& box) const = 0;
	...
};
```

In trace.h, I add a BVHNode class that inherits from the Surface class.  Because BVH involves left and right subtrees, we add the left and right subtrees as a linked list.

```c++
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
```

The hardest part of this data structure is setting it up, and fortunately there is a good example in "Ray Tracing: The Next Week".

```c++
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
```

The left and right subtrees are recursively operated until the shot reaches the leaf node, hitting the overlapping part, and the hit data is outgoing with the hr.

```c++
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
```


BVH speeds up the program greatly.

![image-20220911181153963](https://user-images.githubusercontent.com/82855166/189536909-2701ae19-74c3-4fc2-a993-d2743e7d1593.png)

- ### Antialiasing

![image-20220911182815810](https://user-images.githubusercontent.com/82855166/189536409-8fad3ca0-92c2-478b-914f-e2a963ea0f8e.png)

## Some erroneous images

- The reflection ray calculated from the light emitted by the light instead of the eye causes the picture to be overexposed.

![a8fe9c1b569a269d57c54483ae735b0](https://user-images.githubusercontent.com/82855166/189536421-5fdc59fb-e6e7-4ad0-b849-2be80181e287.jpg)

- After shadow judgment, the background color is returned directly without considering the influence of indirect illumination(reflection).

![image-20220911185016538](https://user-images.githubusercontent.com/82855166/189536430-54870bfe-9f45-4ad0-9416-82f297db829b.png)

- The erroneous triangle bounding box causes some triangles that should be rendered to be lost.

![image-20220911190541275](https://user-images.githubusercontent.com/82855166/189536434-a58fe18d-784f-4a26-bb2b-b2814a6beee5.png)

## results

![balls](https://user-images.githubusercontent.com/82855166/188321733-292f90aa-ffb1-4afa-aa9c-6bac6a71e332.jpg)
![teapot](https://user-images.githubusercontent.com/82855166/188321084-34f5bb4e-d47a-4e90-a634-1b9ffd07d0cc.jpg)
![gears](https://user-images.githubusercontent.com/82855166/188459403-dcfb693d-f192-4601-8673-a2660cad19c3.jpg)
![mount](https://user-images.githubusercontent.com/82855166/188362683-41314b5a-f639-461a-90ff-57bf3dc9a7ae.jpg)
![rings](https://user-images.githubusercontent.com/82855166/188364464-9955e577-abe2-4bc5-94e8-9790b977c397.jpg)
