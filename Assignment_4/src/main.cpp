////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <stack>

// Eigen for matrix operations
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
	Vector3d origin;
	Vector3d direction;
	Ray() { }
	Ray(Vector3d o, Vector3d d) : origin(o), direction(d) { }
};

struct Light {
	Vector3d position;
	Vector3d intensity;
};

struct Intersection {
	Vector3d position;
	Vector3d normal;
	double ray_param;
};

struct Camera {
	bool is_perspective;
	Vector3d position;
	double field_of_view; // between 0 and PI
	double focal_length;
	double lens_radius; // for depth of field
};

struct Material {
	Vector3d ambient_color;
	Vector3d diffuse_color;
	Vector3d specular_color;
	double specular_exponent; // Also called "shininess"

	Vector3d reflection_color;
	Vector3d refraction_color;
	double refraction_index;
};

struct Object {
	Material material;
	virtual ~Object() = default; // Classes with virtual methods should have a virtual destructor!
	virtual bool intersect(const Ray &ray, Intersection &hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
	Vector3d position;
	double radius;

	virtual ~Sphere() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Parallelogram : public Object {
	Vector3d origin;
	Vector3d u;
	Vector3d v;

	virtual ~Parallelogram() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct AABBTree {
	struct Node {
		AlignedBox3d bbox;
		int parent; // Index of the parent node (-1 for root)
		int left; // Index of the left child (-1 for a leaf)
		int right; // Index of the right child (-1 for a leaf)
		int triangle; // Index of the node triangle (-1 for internal nodes)
	};

	std::vector<Node> nodes;
	int root;

	AABBTree() = default; // Default empty constructor
	AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

struct Mesh : public Object {
	MatrixXd vertices; // n x 3 matrix (n points)
	MatrixXi facets; // m x 3 matrix (m triangles)

	AABBTree bvh;

	Mesh() = default; // Default empty constructor
	Mesh(const std::string &filename);
	virtual ~Mesh() = default;
	virtual bool intersect(const Ray &ray, Intersection &hit) override;
};

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

// Read a triangle mesh from an off file
void load_off(const std::string &filename, MatrixXd &V, MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(nv, 3);
	F.resize(nf, 3);
	for (int i = 0; i < nv; ++i) {
		in >> V(i, 0) >> V(i, 1) >> V(i, 2);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);
		assert(s == 3);
	}
}

Mesh::Mesh(const std::string &filename) {
	// Load a mesh from a file (assuming this is a .off file), and create a bvh
	load_off(filename, vertices, facets);
	bvh = AABBTree(vertices, facets);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Implementation
////////////////////////////////////////////////////////////////////////////////

// Bounding box of a triangle
AlignedBox3d bbox_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c) {
	AlignedBox3d box;
	box.extend(a);
	box.extend(b);
	box.extend(c);
	return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F) {//OK
	// Compute the centroids of all the triangles in the input mesh
	MatrixXd centroids(F.rows(), V.cols());
	centroids.setZero();
	for (int i = 0; i < F.rows(); ++i) {
		for (int k = 0; k < F.cols(); ++k) {
			centroids.row(i) += V.row(F(i, k));
		}
		centroids.row(i) /= F.cols();
	}

	// TODO (Assignment 3)

	// Method (1): Top-down approach.
	// Split each set of primitives into 2 sets of roughly equal size,
	// based on sorting the centroids along one direction or another.

	// Method (2): Bottom-up approach.
	// Merge nodes 2 by 2, starting from the leaves of the forest, until only 1 tree is left.

	std::cout << "building AABB tree for mesh ..." << "\n";

	int n = F.rows();
	std::vector<int> S;

	// going through every triangle in the mesh and adding a node for each to the AABB tree
	// along with bounding box
	for (unsigned i = 0; i < n; i ++){
		Node new_node;
		new_node.bbox = bbox_triangle(V.row(F(i, 0)), V.row(F(i, 1)), V.row(F(i, 2)));
		new_node.triangle = i;
		new_node.left = -1;
		new_node.right = -1;
		// adding the node to S to further analyse the closest AABB for merging to the top
		S.push_back(i);
		nodes.push_back(new_node);
	}

	while(S.size() != 1){
		// looping till the size of S is greater than 1
		for (unsigned i = 0; i < S.size() - 1; i ++){
			double min_dist = 100000;
			int closest_box = -1;
			// for each node in S_i comparing the distance with S_j
			for (unsigned j = i + 1; j < S.size(); j ++){
				if (nodes[S[i]].bbox.squaredExteriorDistance(nodes[S[j]].bbox) < min_dist)
				{
					// if the distance is lesser than the previously computed min distance 
					// updating it with the latest closest box
					closest_box = j;
					min_dist = nodes[S[i]].bbox.squaredExteriorDistance(nodes[S[j]].bbox);
					if (min_dist == 0){
						// if the min distance is zero then there is no point in checking any more as
						// there can not be a closer box then this (there maybe another box that is zero but not considered)
						break;
					}
				}
			}
			// creating the new node that merged node i and closest_box
			Node new_node;
			new_node.bbox.extend(nodes[S[i]].bbox);
			new_node.bbox.extend(nodes[S[closest_box]].bbox);
			new_node.left = S[i];
			new_node.right = S[closest_box];
			new_node.triangle = -1;
			nodes.push_back(new_node);
			nodes[S[i]].parent = nodes.size() - 1;
			nodes[S[closest_box]].parent = nodes.size() - 1;

			// adding the latest node to S to be analysed in place of the 
			// i_th node that has already been analysed and merged
			// this ensures that this new node is only considered in the next pass where all
			// the AABB at the lower level of the tree are already merged
			S[i] = nodes.size() - 1;
			// removing the node that has been merged
			S.erase(S.begin() + closest_box);
		}
	}
	// adding the parent paramters to the root node
	nodes[nodes.size() - 1].parent = -1;
	// updating the struct with the index to the parent parameter
	root = nodes.size() - 1;

	std::cout << "Finished building AABB tree for mesh ..." << "\n";
}

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {//OK
	// TODO (Assignment 2)
	double a=ray.direction.dot(ray.direction); //d*d
	double b=2*ray.direction.dot(ray.origin-position); //2d(e-c)
    double c=(ray.origin-position).dot(ray.origin-position)-radius*radius;// (e-c)(e-c)-R*R
	double delta=b*b-4*a*c;
	if(delta>=0){
		double t,t1,t2;
	    if(delta==0){
			t=(-b)/(2*a);
		}
		else{
			t1=(-b-sqrt(delta))/(2*a);
			t2=(-b+sqrt(delta))/(2*a);
			if(t1>=0 && t2>=0){
				if(t1<t2) t=t1;
				else t=t2;
			}
			else if(t1 < 0 && t2 < 0) return false;
			else if(t1>=0 && t2<0) t=t1;
			else t=t2;
		}
	    hit.position=t * ray.direction+t*ray.origin;
	 	hit.normal = (hit.position - position).normalized();//p-c
		return true;
	}
	else return false;  
				
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {//OK
	// TODO (Assignment 2)
	Matrix3d leftEqu;
	Vector3d rightEqu;
	Vector3d uvt;
	for(int i=0;i<3;i++){
		leftEqu(i,0)=u[i];//i=0,(a-b)x; i=1,(a-b)y; i=2,(a-b)z
		leftEqu(i,1)=v[i];//i=0,(a-c)x; i=1,(a-c)y; i=2,(a-c)z
		leftEqu(i,2)=-ray.direction[i];
	}
	rightEqu= ray.origin-origin;
	uvt= leftEqu.colPivHouseholderQr().solve(rightEqu);

	if (uvt[0]<=1 &&uvt[0]>=0 &&uvt[1]<=1&&uvt[1]>=0&&uvt[2]>0) {
		hit.position = uvt[2] * ray.direction + ray.origin;
		hit.normal = (u.cross(v)).normalized();//(b-a)x(c-a)/|(b-a)x(c-a)|
		return true;
	}
	else return false; 
}

// -----------------------------------------------------------------------------

bool intersect_triangle(const Ray &ray, const Vector3d &a, const Vector3d &b, const Vector3d &c, Intersection &hit) {//OK
	// TODO (Assignment 3)
	//
	// Compute whether the ray intersects the given triangle.
	// If you have done the parallelogram case, this should be very similar to it.
	Vector3d u=a-b;
	Vector3d v=c-b;
	Matrix3d leftEqu;
	Vector3d rightEqu;
	Vector3d uvt;
	for(int i=0;i<3;i++){
		leftEqu(i,0)=u[i];//i=0,(a-b)x; i=1,(a-b)y; i=2,(a-b)z
		leftEqu(i,1)=v[i];//i=0,(a-c)x; i=1,(a-c)y; i=2,(a-c)z
		leftEqu(i,2)=-ray.direction[i];
	}
	rightEqu=ray.origin-b;
	uvt= leftEqu.colPivHouseholderQr().solve(rightEqu);
	if (uvt[0]>=0 &&uvt[1]>=0 &&uvt[0]+uvt[1]<=1&&uvt[2]>=0) {
		hit.position = uvt[2] * ray.direction + ray.origin;
		hit.normal = (-u.cross(v)).normalized();//(b-a)x(c-a)/|(b-a)x(c-a)|
		return true;
	}
	else return false; 
}

bool intersect_box(const Ray &ray, const AlignedBox3d &box) {
	// TODO (Assignment 3)
	//
	// Compute whether the ray intersects the given box.
	// There is no need to set the resulting normal and ray parameter, since
	// we are not testing with the real surface here anyway.
	double t_max=10000;
	double t_min=0;
	double t1;
	double t2;

	for(int i=0;i<3;i++){
		t1=(box.min()[i]-ray.origin[i])*ray.direction[i];
		t2=(box.max()[i]-ray.origin[i])*ray.direction[i];
		if(ray.direction[i]<0){
			double temp=t2;
			t2=t1;
			t1=temp;
		}
		double t_max=std::max(t_max,std::max(t1,t2));
		double t_min=std::min(t_min,std::min(t1,t2));

	}
	if (t_max <= t_min){
			return false;
	} 
	return true; 
}

bool Mesh::intersect(const Ray &ray, Intersection &closest_hit) {//OK
	// TODO (Assignment 3)

	// Method (1): Traverse every triangle and return the closest hit.

	// Method (2): Traverse the BVH tree and test the intersection with a
	// triangles at the leaf nodes that intersects the input ray.

	Intersection current_hit;
	int closest_index = -1;
	bool speed_up = true; // set to false for method (1)
	if(!speed_up){
		// Method (1): Traverse every triangle and return the closest hit.
		for (unsigned i = 0; i < facets.rows(); i++)
		{
			if (intersect_triangle(ray, vertices.row(facets(i, 0)), vertices.row(facets(i, 1)), vertices.row(facets(i, 2)), current_hit))
			{
				if (closest_index >= 0)
				{
					if ((ray.origin - current_hit.position).squaredNorm() < (ray.origin - closest_hit.position).squaredNorm())
					{
						closest_hit = current_hit;
						closest_index = i;
					}
				}
				else
				{
					closest_index = i;
					closest_hit = current_hit;
				}
			}
		}
	}
	else{
		// Method (2): Traverse the BVH tree and test the intersection with a
		// triangles at the leaf nodes that intersects the input ray.
		std::vector<int> S;
		S.push_back(bvh.root);

		while (S.size() > 0){
			for(unsigned i = 0; i < S.size(); i ++){
				// checking ray intersect AABB 
				if(intersect_box(ray, bvh.nodes[S[i]].bbox)){
					if (bvh.nodes[S[i]].triangle == -1){
						// if node is not a leaf then adding children to the search of the tree
						S.push_back(bvh.nodes[S[i]].left);
						S.push_back(bvh.nodes[S[i]].right);
					}
					else{
						// if its a leaf node then checking if the ray hits the triangle
						int index = bvh.nodes[S[i]].triangle;
						if( intersect_triangle(ray, vertices.row(facets(index, 0)), vertices.row(facets(index, 1)), vertices.row(facets(index, 2)), current_hit)){
							if (closest_index >= 0){
								if ((ray.origin - current_hit.position).squaredNorm() < (ray.origin - closest_hit.position).squaredNorm())
								{
									// checking if the ray has hit another triangle that is farther than the current one in
									// in which case it is updated to this triangle
									closest_hit = current_hit;
									closest_index = i;
								}
							}
							else
							{
								closest_index = i;
								closest_hit = current_hit;
							}
						}
					}
				}
				// removing tree since it has been visited and analysed
				S.erase(S.begin() + i);
			}
		}
	}
	if (closest_index < 0){
			// Returning false since ray does not hit any triangle in the mesh
			return false;
		}
		else
		{
			// returning true becuase ray hits a triangle in the mesh
			return true;
		}
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light, const Intersection &hit);
Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce);

// -----------------------------------------------------------------------------

Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &obj, const Intersection &hit, int max_bounce) {
	// Material for hit object
	const Material &mat = obj.material;

	// Ambient light contribution
	Vector3d ambient_color = obj.material.ambient_color.array() * scene.ambient_light.array();

	// Punctual lights contribution (direct lighting)
	Vector3d lights_color(0, 0, 0);
	double offset=0.001;
	for (const Light &light : scene.lights) {
		Vector3d Li = (light.position - hit.position).normalized();
		Vector3d N = hit.normal;
		Vector3d Middle=((light.position - hit.position) - ray.direction).normalized();

		// TODO (Assignment 2, shadow rays)
		Ray shadow_ray;
		shadow_ray.origin = hit.position;
	    shadow_ray.direction = (light.position - hit.position);
	    shadow_ray.origin += offset*shadow_ray.direction;
		if(!is_light_visible(scene, shadow_ray,light,hit)){
			continue;
		} 
		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// TODO (Assignment 2, specular contribution)
		Vector3d specular=mat.specular_color*std::pow(std::max(N.dot(Middle), 0.0), mat.specular_exponent);

		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}

	// TODO (Assignment 2, reflected ray)
	Vector3d reflection_color(0, 0, 0);
	
	// TODO (Assignment 2, refracted ray)
	Vector3d refraction_color(0, 0, 0);
	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// TODO (Assignment 2, find nearest hit)
	Intersection temp_hit;
	for(unsigned i=0;i<scene.objects.size();i++){
		if(scene.objects[i].get()->intersect(ray,temp_hit)){
			if(closest_index<0){
				closest_index=i;
				closest_hit=temp_hit;
			}
			else{
				if((ray.origin-closest_hit.position).squaredNorm()>(ray.origin-temp_hit.position).squaredNorm()){
					closest_index=i;
				    closest_hit=temp_hit;
				}
			}
		}
	}

	if (closest_index < 0) {
		// Return a NULL pointer
		return nullptr;
	} else {
		// Return a pointer to the hit object. Don't forget to set 'closest_hit' accordingly!
		return scene.objects[closest_index].get();
	}
}

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light, const Intersection &hit) {
	// TODO (Assignment 2, shadow ray)
	/* for (const ObjectPtr object : scene.objects) {
		Intersection temphit;
		if (object->intersect(ray, temphit)) {
			return false; //this light will not contribute to the color.
		}
	}
	return true;  */
	Intersection closest_hit;
	if (Object * obj = find_nearest_object(scene, ray, closest_hit)){
		// ensuring that the intersection of the shadow ray is before the ligth source 
		// if the object lies behind the light source, then light is still visible despite the intersection
		if ((closest_hit.position - hit.position).squaredNorm() < (light.position - hit.position).squaredNorm()){
			return false;
		}
		else{
			return true;
		}
	}
	else{
		return true;
	}
}

Vector3d shoot_ray(const Scene &scene, const Ray &ray, int max_bounce) {
	Intersection hit;
	if (Object * obj = find_nearest_object(scene, ray, hit)) {
		// 'obj' is not null and points to the object of the scene hit by the ray
		return ray_color(scene, ray, *obj, hit, max_bounce);
	} else {
		// 'obj' is null, we must return the background color
		return scene.background_color;
	}
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene &scene) {
	std::cout << "Simple ray tracer." << std::endl;

	int w = 640;
	int h = 480;
	MatrixXd R = MatrixXd::Zero(w, h);
	MatrixXd G = MatrixXd::Zero(w, h);
	MatrixXd B = MatrixXd::Zero(w, h);
	MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

	// The camera always points in the direction -z
	// The sensor grid is at a distance 'focal_length' from the camera center,
	// and covers an viewing angle given by 'field_of_view'.//OK
	double aspect_ratio = double(w) / double(h);
	double scale_y =2*scene.camera.focal_length*tan(scene.camera.field_of_view/2.0); // TODO: Stretch the pixel grid by the proper amount here
	double scale_x =aspect_ratio*scale_y;//

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	for (unsigned i = 0; i < w; ++i) {
		std::cout << std::fixed << std::setprecision(2);
		std::cout << "Ray tracing: " << (100.0 * i) / w << "%\r" << std::flush;
		for (unsigned j = 0; j < h; ++j) {
			// TODO (Assignment 2, depth of field)
			int number_ray=1;
			double r=0;
			for (int k = 0; k <number_ray; k++) {
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;

			// Prepare the ray
			Ray ray;

			if (scene.camera.is_perspective) {
				// Perspective camera
				// TODO (Assignment 2, perspective camera)
				ray.origin=scene.camera.position; 
					
					double x,y;
				    x= r*(double)rand()/RAND_MAX*2.0-1.0;;
				    y= r*(double)rand()/RAND_MAX*2.0-1.0;;

				    ray.origin[0]+=scene.camera.lens_radius*x;
				    ray.origin[1]+=scene.camera.lens_radius*y;
                    ray.direction=shift-ray.origin; 
			} else {
				// Orthographic camera
				ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
				ray.direction = Vector3d(0, 0, -1);
			}

			int max_bounce = 5;
			Vector3d C = shoot_ray(scene, ray, max_bounce);
			R(i, j) += C(0)/number_ray;
			G(i, j) += C(1)/number_ray;
			B(i, j) += C(2)/number_ray;
			A(i, j) = 1;

			}
		}
	}

	std::cout << "Ray tracing: 100%  " << std::endl;

	// Save to png
	const std::string filename("raytrace.png");
	write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string &filename) {
	Scene scene;

	// Load json data from scene file
	json data;
	std::ifstream in(filename);
	in >> data;

	// Helper function to read a Vector3d from a json array
	auto read_vec3 = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};

	// Read scene info
	scene.background_color = read_vec3(data["Scene"]["Background"]);
	scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

	// Read camera info
	scene.camera.is_perspective = data["Camera"]["IsPerspective"];
	scene.camera.position = read_vec3(data["Camera"]["Position"]);
	scene.camera.field_of_view = data["Camera"]["FieldOfView"];
	scene.camera.focal_length = data["Camera"]["FocalLength"];
	scene.camera.lens_radius = data["Camera"]["LensRadius"];

	// Read materials
	for (const auto &entry : data["Materials"]) {
		Material mat;
		mat.ambient_color = read_vec3(entry["Ambient"]);
		mat.diffuse_color = read_vec3(entry["Diffuse"]);
		mat.specular_color = read_vec3(entry["Specular"]);
		mat.reflection_color = read_vec3(entry["Mirror"]);
		mat.refraction_color = read_vec3(entry["Refraction"]);
		mat.refraction_index = entry["RefractionIndex"];
		mat.specular_exponent = entry["Shininess"];
		scene.materials.push_back(mat);
	}

	// Read lights
	for (const auto &entry : data["Lights"]) {
		Light light;
		light.position = read_vec3(entry["Position"]);
		light.intensity = read_vec3(entry["Color"]);
		scene.lights.push_back(light);
	}

	// Read objects
	for (const auto &entry : data["Objects"]) {
		ObjectPtr object;
		if (entry["Type"] == "Sphere") {
			auto sphere = std::make_shared<Sphere>();
			sphere->position = read_vec3(entry["Position"]);
			sphere->radius = entry["Radius"];
			object = sphere;
		} else if (entry["Type"] == "Parallelogram") {
			// TODO//OK
			auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
            parallelogram->u = read_vec3(entry["u"]);
            parallelogram->v = read_vec3(entry["v"]);
		} else if (entry["Type"] == "Mesh") {
			// Load mesh from a file
			std::string filename = std::string(DATA_DIR) + entry["Path"].get<std::string>();
			object = std::make_shared<Mesh>(filename);
		}
		object->material = scene.materials[entry["Material"]];
		scene.objects.push_back(object);
	}

	return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	}
	Scene scene = load_scene(argv[1]);
	render_scene(scene);
	return 0;
} 

