# Assignment01_extra features



# Assignment02_ray tracer

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

![image-20220911094912949](C:\Users\米高周敦\AppData\Roaming\Typora\typora-user-images\image-20220911094912949.png)

## Step 3 Check for shadows

Before calculating the normal illumination, the vector between the intersection point and the light source is checked to see if it intersects the object in the scene, and the distance between it and the light source is checked. The intersect function is passed a HitRecord dummy, and the initial value of t0 is set to be greater than 0 to prevent the computed vector between the intersection and the light source from intersecting itself, while the initial value of t1 is set to a sufficiently large number.

<img src="C:\Users\米高周敦\Desktop\图片\shadows.jpg" alt="shadows" style="zoom: 50%;" />

## Step 4 Computing reflection color

Reflection shows the wonders of ray tracing! We assume that the light bounces multiple times across the surface of an object in the scene, and when calculating the color of a point, we simply add up the illumination contribution from each bounce.

To control the number of times the light bounces, add a check on whether hr.rayDepth is less than maxraydepth.When a new reflection is obtained, hr.raydepth+1,and when hr.raydepth exceeds maxraydepth, the ray will no longer be ejected.

<img src="C:\Users\米高周敦\Desktop\图片\reflection.jpg" alt="reflection" style="zoom:50%;" />

## Step 5 Computing refraction color

I am working on the computing refraction color part, now I have a Fresnel function to calculate the reflectance and transmittance, a function to get the refraction ray, but the obtained data still needs to be verified, I tried to add the relevant logic to the code to test.

There are some erroneous images here, maybe it is a logic error or the input of the wrong direction of the ray, I will continue to fix and update here in the future.

<img src="C:\Users\米高周敦\AppData\Roaming\Typora\typora-user-images\image-20220911181648273.png" alt="image-20220911181648273" style="zoom:33%;" />

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

<img src="C:\Users\米高周敦\AppData\Roaming\Typora\typora-user-images\image-20220911181153963.png" alt="image-20220911181153963" style="zoom: 33%;" />

- ### Antialiasing

![image-20220911182815810](C:\Users\米高周敦\AppData\Roaming\Typora\typora-user-images\image-20220911182815810.png)

## Some erroneous images

- The reflection ray calculated from the light emitted by the light instead of the eye causes the picture to be overexposed.

<img src="C:\Users\米高周敦\AppData\Local\Temp\WeChat Files\a8fe9c1b569a269d57c54483ae735b0.jpg" alt="a8fe9c1b569a269d57c54483ae735b0" style="zoom: 50%;" />

- After shadow judgment, the background color is returned directly without considering the influence of indirect illumination(reflection).

<img src="C:\Users\米高周敦\AppData\Roaming\Typora\typora-user-images\image-20220911185016538.png" alt="image-20220911185016538" style="zoom: 33%;" />

- The erroneous triangle bounding box causes some triangles that should be rendered to be lost.

<img src="C:\Users\米高周敦\AppData\Roaming\Typora\typora-user-images\image-20220911190541275.png" alt="image-20220911190541275" style="zoom: 50%;" />
