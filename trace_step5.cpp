#include "trace.H"
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>

//#include <getopt.h>
#ifdef __APPLE__
#define MAX std::numeric_limits<double>::max()
#else
//#include <values.h>
#define MAX DBL_MAX
#define M_PI 3.1415926
#endif

// return the determinant of the matrix with columns a, b, c.
double det(const SlVector3& a, const SlVector3& b, const SlVector3& c) {
	return a[0] * (b[1] * c[2] - c[1] * b[2]) +
		b[0] * (c[1] * a[2] - a[1] * c[2]) +
		c[0] * (a[1] * b[2] - b[1] * a[2]);
}

inline double sqr(double x) { return x * x; }

inline SlVector3 reflect(const SlVector3& I, const SlVector3& N) {
	return 2 * dot(I, N) * N - I;
};

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

bool Triangle::boundingBox(AABB& box) const
{
	SlVector3 _min, _max;
	_min[0] = ffmin(a.x(), ffmin(b.x(), c.x()));
	_min[1] = ffmin(a.y(), ffmin(b.y(), c.y()));
	_min[2] = ffmin(a.z(), ffmin(b.z(), c.z()));

	// 0.1 is added to prevent the box from having no volume (the model is a plane parallel to the axes).
	_max[0] = ffmax(a.x(), ffmax(b.x(), c.x())) + 0.1;
	_max[1] = ffmax(a.y(), ffmax(b.y(), c.y())) + 0.1;
	_max[2] = ffmax(a.z(), ffmax(b.z(), c.z())) + 0.1;

	box = AABB(_min, _max);
	return true;
}

bool Sphere::boundingBox(AABB& box) const
{
	box = AABB(c - SlVector3(rad, rad, rad), c + SlVector3(rad, rad, rad));
	return true;
}

bool Triangle::intersect(const Ray& r, double t0, double t1, HitRecord& hr) const {

	// Step 1 Ray-triangle test
	// Möller-Trumbore algorithm
	SlVector3 E1 = b - a;
	SlVector3 E2 = c - a;
	SlVector3 S = r.e - a;
	SlVector3 S1 = cross(r.d, E2);
	SlVector3 S2 = cross(S, E1);
	float coeff = 1.0 / dot(S1, E1);
	float t = coeff * dot(S2, E2);
	float b1 = coeff * dot(S1, S);
	float b2 = coeff * dot(S2, r.d);
	if (t > t0 && t < t1)
	{
		if (t >= 0 && b1 >= 0 && b2 >= 0 && (1 - b1 - b2) >= 0)
		{
			hr.t = t;
			hr.p = r.e + t * r.d;
			hr.beta = b1;
			hr.gamma = b2;
			hr.alpha = 1 - hr.beta - hr.gamma;
			hr.v = r.e;
			hr.raydepth = r.depth;
			hr.n = cross(E1, E2);
			normalize(hr.n);
			hr.f = mat;
			return true;
		}
	}
	return false;
}

bool TrianglePatch::intersect(const Ray& r, double t0, double t1, HitRecord& hr) const {
	bool temp = Triangle::intersect(r, t0, t1, hr);
	if (temp) {
		hr.n = hr.alpha * n1 + hr.beta * n2 + hr.gamma * n3;
		normalize(hr.n);
	}
	return temp;

}

bool Sphere::intersect(const Ray& r, double t0, double t1, HitRecord& hr) const {
	// Step 1 Sphere-triangle test
	SlVector3 x = r.e - c;
	float A = sqrMag(r.d);
	float B = 2 * dot(x, r.d);
	float C = sqrMag(x) - sqr(rad);
	float delta = sqr(B) - 4 * A * C;
	if (delta < 0)return false;
	else {
		float t11 = 0.5 * (-B + sqrt(delta)) / A;
		float t12 = 0.5 * (-B - sqrt(delta)) / A;
		float t = t11;
		if (t11 > 0 && t11 < t12)t = t11;
		if (t12 > 0 && t12 < t11)t = t12;
		if (t > t0 && t < t1) {
			hr.t = t;
			hr.p = r.e + t * r.d;
			hr.v = r.e;
			hr.n = hr.p - c;
			normalize(hr.n);
			hr.raydepth = r.depth;
			hr.f = mat;
			return true;
		}
		return false;
	}
}


Tracer::Tracer(const std::string& fname) {
	std::ifstream in(fname.c_str(), std::ios_base::in);
	std::string line;
	char ch;
	Fill fill;
	bool coloredlights = false;
	while (in) {
		getline(in, line);
		switch (line[0]) {
		case 'b': {
			std::stringstream ss(line);
			ss >> ch >> bcolor[0] >> bcolor[1] >> bcolor[2];
			break;
		}

		case 'v': {
			getline(in, line);
			std::string junk;
			std::stringstream fromss(line);
			fromss >> junk >> eye[0] >> eye[1] >> eye[2];

			getline(in, line);
			std::stringstream atss(line);
			atss >> junk >> at[0] >> at[1] >> at[2];

			getline(in, line);
			std::stringstream upss(line);
			upss >> junk >> up[0] >> up[1] >> up[2];

			getline(in, line);
			std::stringstream angless(line);
			angless >> junk >> angle;

			getline(in, line);
			std::stringstream hitherss(line);
			hitherss >> junk >> hither;

			getline(in, line);
			std::stringstream resolutionss(line);
			resolutionss >> junk >> res[0] >> res[1];
			break;
		}

		case 'p': {
			bool patch = false;
			std::stringstream ssn(line);
			unsigned int nverts;
			if (line[1] == 'p') {
				patch = true;
				ssn >> ch;
			}
			ssn >> ch >> nverts;
			std::vector<SlVector3> vertices;
			std::vector<SlVector3> normals;
			for (unsigned int i = 0; i < nverts; i++) {
				getline(in, line);
				std::stringstream ss(line);
				SlVector3 v, n;
				if (patch) ss >> v[0] >> v[1] >> v[2] >> n[0] >> n[1] >> n[2];
				else ss >> v[0] >> v[1] >> v[2];
				vertices.push_back(v);
				normals.push_back(n);
			}
			bool makeTriangles = false;
			if (vertices.size() == 3) {
				if (patch) {
					//surfaces.push_back(std::pair<Surface*, Fill>(new TrianglePatch(vertices[0], vertices[1], vertices[2],
					//	normals[0], normals[1], normals[2]), fill));
					list.push_back(new TrianglePatch(vertices[0], vertices[1], vertices[2],
						normals[0], normals[1], normals[2], fill));
				}
				else {
					//surfaces.push_back(std::pair<Surface*, Fill>(new Triangle(vertices[0], vertices[1], vertices[2]), fill));
					list.push_back(new Triangle(vertices[0], vertices[1], vertices[2], fill));
				}
			}
			else if (vertices.size() == 4) {
				SlVector3 n0 = cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
				SlVector3 n1 = cross(vertices[2] - vertices[1], vertices[3] - vertices[1]);
				SlVector3 n2 = cross(vertices[3] - vertices[2], vertices[0] - vertices[2]);
				SlVector3 n3 = cross(vertices[0] - vertices[3], vertices[1] - vertices[3]);
				if (dot(n0, n1) > 0 && dot(n0, n2) > 0 && dot(n0, n3) > 0) {
					makeTriangles = true;
					if (patch) {
						//surfaces.push_back(std::pair<Surface*, Fill>(new TrianglePatch(vertices[0], vertices[1], vertices[2],
						//	normals[0], normals[1], normals[2]), fill));
						list.push_back(new TrianglePatch(vertices[0], vertices[1], vertices[2],
							normals[0], normals[1], normals[2], fill));
						//surfaces.push_back(std::pair<Surface*, Fill>(new TrianglePatch(vertices[0], vertices[2], vertices[3],
						//	normals[0], normals[2], normals[3]), fill));
						list.push_back(new TrianglePatch(vertices[0], vertices[2], vertices[3],
							normals[0], normals[2], normals[3], fill));
					}
					else {
						//surfaces.push_back(std::pair<Surface*, Fill>(new Triangle(vertices[0], vertices[1], vertices[2]), fill));
						list.push_back(new Triangle(vertices[0], vertices[1], vertices[2], fill));
						//surfaces.push_back(std::pair<Surface*, Fill>(new Triangle(vertices[0], vertices[2], vertices[3]), fill));
						list.push_back(new Triangle(vertices[0], vertices[2], vertices[3], fill));
					}
				}
				if (!makeTriangles) {
					std::cerr << "I didn't make triangles.  Poly not flat or more than quad.\n";
				}
			}
			break;
		}

		case 's': {
			std::stringstream ss(line);
			SlVector3 c;
			double r;
			ss >> ch >> c[0] >> c[1] >> c[2] >> r;
			//surfaces.push_back(std::pair<Surface*, Fill>(new Sphere(c, r), fill));
			list.push_back(new Sphere(c, r, fill));
			break;
		}

		case 'f': {
			std::stringstream ss(line);
			ss >> ch >> fill.color[0] >> fill.color[1] >> fill.color[2] >> fill.kd >> fill.ks >> fill.shine >> fill.t >> fill.ior;
			break;
		}

		case 'l': {
			std::stringstream ss(line);
			Light l;
			ss >> ch >> l.p[0] >> l.p[1] >> l.p[2];
			if (!ss.eof()) {
				ss >> l.c[0] >> l.c[1] >> l.c[2];
				coloredlights = true;
			}
			lights.push_back(l);
			break;
		}

		default:
			break;
		}
	}
	if (!coloredlights) for (unsigned int i = 0; i < lights.size(); i++) lights[i].c = 1.0 / sqrt(lights.size());
	im = new SlVector3[res[0] * res[1]];
	shadowbias = 1e-6;
	samples = 1;
	aperture = 0.0;
	bvh_tree = BVHNode(list);
}

Tracer::~Tracer() {
	if (im) delete[] im;
	//for (unsigned int i = 0; i < surfaces.size(); i++) delete surfaces[i].first;
	for (unsigned int i = 0; i < list.size(); i++) delete list[i];
}

SlVector3 Tracer::shade(const HitRecord& hr) const {
	if (color) return hr.f.color;

	SlVector3 color(0.0);
	HitRecord dummy;

	for (unsigned int i = 0; i < lights.size(); i++) {
		const Light& light = lights[i];
		bool shadow = false;

		//Step 3 Check for shadows here
		SlVector3 dummyLigDir = light.p - hr.p;
		Ray dummyLig(hr.p, dummyLigDir);

		if (bvh_tree.intersect(dummyLig, 0.001, MAX, dummy)) {
			shadow = true;
		}

		if (!shadow) {
			//Step 2 do shading here
			SlVector3 viewDir = hr.v - hr.p;
			SlVector3 rayDir = light.p - hr.p;
			SlVector3 reflectDir = reflect(rayDir, hr.n);
			normalize(viewDir);
			normalize(rayDir);
			normalize(reflectDir);
			//ambient
			SlVector3 ambient = 0.1f * light.c;
			//diffuse		
			float diff = fmax(dot(hr.n, rayDir), 0.0);
			SlVector3 diffuse = hr.f.kd * diff * light.c;
			//specular
			float spec = pow(fmax(dot(viewDir, reflectDir), 0.0), hr.f.shine);
			SlVector3 specular = hr.f.ks * spec * light.c;
			//ambient + diffuse + specular
			color += (ambient + diffuse + specular) * hr.f.color;
		}
	}
	// Step 4 Add code for computing reflection color here
	if (hr.raydepth < maxraydepth)
	{
		SlVector3 i = hr.v - hr.p;
		normalize(i);
		SlVector3 reflectionDir = reflect(i, hr.n);
		Ray reflection(hr.p, reflectionDir, hr.raydepth + 1);
		SlVector3 reflectionCol = hr.f.ks * trace(reflection, 0.001, MAX);
		// Step 5 Add code for computing refraction color here
		SlVector3 refractionDir(0.0);
		double k = 1;
		if (refract(i, hr.n, hr.f.ior, refractionDir, k))
		{
			Ray refraction(hr.p, refractionDir, hr.raydepth + 1);
			SlVector3 refractionCol = trace(refraction, 0.001, MAX);
			color += reflectionCol * k + refractionCol * (1 - k);
		}
		else
			color += reflectionCol;
	}
	return color;
}

SlVector3 Tracer::trace(const Ray& r, double t0, double t1) const {
	HitRecord hr;
	SlVector3 color(bcolor);
	bool hit = false;
	// Step 1 See what a ray hits
	if (bvh_tree.intersect(r, t0, t1, hr))hit = true;
	if (hit) color = shade(hr);
	return color;
}

void Tracer::traceImage() {
	// set up coordinate system
	SlVector3 w = eye - at;
	w /= mag(w);
	SlVector3 u = cross(up, w);
	normalize(u);
	SlVector3 v = cross(w, u);
	normalize(v);

	double d = mag(eye - at);
	double h = tan((M_PI / 180.0) * (angle / 2.0)) * d;
	double l = -h;
	double r = h;
	double b = h;
	double t = -h;

	SlVector3* pixel = im;

	for (unsigned int j = 0; j < res[1]; j++) {
		for (unsigned int i = 0; i < res[0]; i++, pixel++) {

			SlVector3 result(0.0, 0.0, 0.0);

			for (int k = 0; k < samples; k++) {

				double rx = 1.1 * rand() / RAND_MAX;
				double ry = 1.1 * rand() / RAND_MAX;

				double x = l + (r - l) * (i + rx) / res[0];
				double y = b + (t - b) * (j + ry) / res[1];
				SlVector3 dir = -d * w + x * u + y * v;

				Ray r(eye, dir);
				normalize(r.d);
				result += trace(r, hither, MAX);

			}
			(*pixel) = result / samples;
		}
	}
}

void Tracer::writeImage(const std::string& fname) {
#ifdef __APPLE__
	std::ofstream out(fname, std::ios::out | std::ios::binary);
#else
	std::ofstream out(fname.c_str(), std::ios_base::binary);
#endif
	out << "P6" << "\n" << res[0] << " " << res[1] << "\n" << 255 << "\n";
	SlVector3* pixel = im;
	char val;
	for (unsigned int i = 0; i < res[0] * res[1]; i++, pixel++) {
		val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[0])) * 255.0);
		out.write(&val, sizeof(unsigned char));
		val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[1])) * 255.0);
		out.write(&val, sizeof(unsigned char));
		val = (unsigned char)(std::min(1.0, std::max(0.0, (*pixel)[2])) * 255.0);
		out.write(&val, sizeof(unsigned char));
	}
	out.close();
}


int main(int argc, char* argv[]) {
	int c;
	double aperture = 0.0;
	int samples = 1;
	int maxraydepth = 5;
	bool color = false;
	/*while ((c = getopt(argc, argv, "a:s:d:c")) != -1) {
		switch (c) {
		case 'a':
			aperture = atof(optarg);
			break;
		case 's':
			samples = atoi(optarg);
			break;
		case 'c':
			color = true;
			break;
		case 'd':
			maxraydepth = atoi(optarg);
			break;
		default:
			abort();
		}
	}

	if (argc - optind != 2) {
		std::cout << "usage: trace [opts] input.nff output.ppm" << std::endl;
		for (unsigned int i = 0; i < argc; i++) std::cout << argv[i] << std::endl;
		exit(0);
	}*/
	Tracer tracer(argv[1]);
	tracer.aperture = aperture;
	tracer.samples = samples;
	tracer.color = color;
	tracer.maxraydepth = maxraydepth;
	tracer.traceImage();
	tracer.writeImage(argv[2]);
};