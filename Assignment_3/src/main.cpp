////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <gif.h>

// Eigen for matrix operations
#include <Eigen/Dense>

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

struct Scene {
	Vector3d background_color;
	Vector3d ambient_light;

	Camera camera;
	std::vector<Material> materials;
	std::vector<Light> lights;
	std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray &ray, Intersection &hit) {
	// TODO:
	//
	// Compute the intersection between the ray and the sphere
	// If the ray hits the sphere, set the result of the intersection in the
	// struct 'hit'
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
			t1=(-b+sqrt(delta))/(2*a);
			t2=(-b-sqrt(delta))/(2*a);
			if(t1>=0 && t2>=0){
				if(t1<t2) t=t1;
				else t=t2;
			}
			else if(t1 < 0 && t2 < 0) return false;
			else if(t1>=0 && t2<0) t=t1;
			else t=t2;

		}
	    hit.position=t * ray.direction+ray.origin;
	 	hit.normal = (hit.position - position).normalized();//p-c
		return true;
	}
	else return false; 
	
}

bool Parallelogram::intersect(const Ray &ray, Intersection &hit) {
	// TODO
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

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene &scene, const Ray &ray, const Object &object, const Intersection &hit, int max_bounce);
Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit);
bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light);
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

		// TODO: Shoot a shadow ray to determine if the light should affect the intersection point
		Ray shadow_ray(hit.position + offset*Li , Li);
		if(!is_light_visible(scene, shadow_ray,light)){
			continue;
		}
		// Diffuse contribution
		Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

		// TODO: Specular contribution
		Vector3d specular=mat.specular_color*std::pow(std::max(N.dot(((light.position - hit.position) - ray.direction).normalized()), 0.0), mat.specular_exponent);

		// Attenuate lights according to the squared distance to the lights
		Vector3d D = light.position - hit.position;
		lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
	}

	// TODO: Compute the color of the reflected ray and add its contribution to the current point color.
	Vector3d reflection_color(0, 0, 0);
    Ray reflected_ray;
	Intersection reflected_hit;
	int reflected_bounce=max_bounce;
	reflected_ray.origin=hit.position;
	reflected_ray.direction=ray.direction-2 *(hit.normal.dot(ray.direction)) * hit.normal;
	reflected_ray.origin += offset*reflected_ray.direction;
	
	for(int i=0;i<reflected_bounce;i++){

		if(Object * obj = find_nearest_object(scene, reflected_ray, reflected_hit)){
			for (const Light &light : scene.lights) {
			Vector3d Li = (light.position - reflected_hit.position).normalized();
			Vector3d N = reflected_hit.normal;

			Ray shadow_ray;
			
			shadow_ray.origin=reflected_hit.position;
			shadow_ray.direction=(light.position-reflected_hit.position);
			shadow_ray.origin=shadow_ray.origin+offset*shadow_ray.direction;
			
			if(!is_light_visible(scene, shadow_ray,light)){
				continue;
			}
			// Diffuse contribution
			Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

			// TODO: Specular contribution
			Vector3d specular=mat.specular_color*std::pow(std::max(N.dot(((light.position - reflected_hit.position) - reflected_ray.direction).normalized()), 0.0), max_bounce);

			// Attenuate lights according to the squared distance to the lights
			Vector3d D = light.position - reflected_hit.position;
			lights_color += (diffuse +specular).cwiseProduct(light.intensity) /  D.squaredNorm();
		    }
			reflected_ray.origin=reflected_hit.position;
			reflected_ray.direction=reflected_ray.direction-2 *(reflected_hit.normal.dot(reflected_ray.direction)) * reflected_hit.normal;
			reflected_ray.origin += offset*reflected_ray.direction;

		}
		else break;
	}  

	// TODO: Compute the color of the refracted ray and add its contribution to the current point color.
	//       Make sure to check for total internal reflection before shooting a new ray.
	Vector3d refraction_color(0, 0, 0);
	Ray refraction_ray;
	Intersection refraction_hit;
    int refraction_bounce=max_bounce;
	refraction_ray.origin=hit.position;
	const double xita = mat.refraction_index;
	refraction_ray.direction = xita*ray.direction + hit.normal*(xita*ray.direction.dot(hit.normal)+sqrt(1-sqrt(xita)*(1-sqrt(ray.direction.dot(hit.normal)))));
	refraction_ray.origin += offset*refraction_ray.direction;
	for(int i=0;i<refraction_bounce;i++){
		if(Object * obj = find_nearest_object(scene, refraction_ray, refraction_hit)){
			for (const Light &light : scene.lights) {
			Vector3d Li = (light.position - refraction_hit.position).normalized();
			Vector3d N = refraction_hit.normal;

			Ray shadow_ray;
			
			shadow_ray.origin=refraction_hit.position;
			shadow_ray.direction=(light.position-refraction_hit.position);
			shadow_ray.origin=shadow_ray.origin+offset*shadow_ray.direction;
			if(!is_light_visible(scene, shadow_ray,light)){
				continue;
			}
			// Diffuse contribution
			Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

			Vector3d specular=mat.specular_color*std::pow(std::max(N.dot(((light.position - reflected_hit.position) - reflected_ray.direction).normalized()), 0.0), max_bounce);

			// Attenuate lights according to the squared distance to the lights
			Vector3d D = light.position - refraction_hit.position;
			lights_color += (diffuse + specular).cwiseProduct(light.intensity) /  D.squaredNorm();
		    }
			refraction_ray.origin=refraction_hit.position;
			refraction_ray.direction = xita*refraction_ray.direction + refraction_hit.normal*(xita*refraction_ray.direction.dot(refraction_hit.normal)+sqrt(1-sqrt(xita)*(1-sqrt(refraction_ray.direction.dot(refraction_hit.normal)))));
			refraction_ray.origin += offset*refraction_ray.direction;

		}
		else break;
		
	}  
	// Rendering equation
	Vector3d C = ambient_color + lights_color + reflection_color + refraction_color;

	return C;
}

// -----------------------------------------------------------------------------

Object * find_nearest_object(const Scene &scene, const Ray &ray, Intersection &closest_hit) {
	int closest_index = -1;
	// TODO:
	//
	// Find the object in the scene that intersects the ray first
	// The function must return 'nullptr' if no object is hit, otherwise it must
	// return a pointer to the hit object, and set the parameters of the argument
	// 'hit' to their expected values.
	Intersection temp_hit;
	for(int i=0;i<scene.objects.size();i++){
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

bool is_light_visible(const Scene &scene, const Ray &ray, const Light &light) {
	// TODO: Determine if the light is visible here
	//Intersection closest_hit;
	for (const ObjectPtr object : scene.objects) {
		Intersection temphit;
		if (object->intersect(ray, temphit)) {
			return false; //this light will not contribute to the color.
		}
	}
	return true;
	//return find_nearest_object(scene,ray, closest_hit)==nullptr;
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
	// and covers an viewing angle given by 'field_of_view'.
	double aspect_ratio = double(w) / double(h); 
	double scale_y = 2*scene.camera.focal_length*tan(scene.camera.field_of_view/2.0); // TODO: Stretch the pixel grid by the proper amount here //P1
	double scale_x =aspect_ratio*scale_y; //

	// The pixel grid through which we shoot rays is at a distance 'focal_length'
	// from the sensor, and is scaled from the canonical [-1,1] in order
	// to produce the target field of view.
	Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
	Vector3d x_displacement(2.0/w*scale_x, 0, 0);
	Vector3d y_displacement(0, -2.0/h*scale_y, 0);

	
	int number_ray=5;
	for (unsigned i = 0; i < w; ++i) {
		for (unsigned j = 0; j < h; ++j) {

			// TODO: Implement depth of field
			
			for (unsigned k = 0; k <number_ray; k++) {
			Vector3d shift = grid_origin + (i+0.5)*x_displacement + (j+0.5)*y_displacement;
			// Prepare the ray
			Ray ray;

			if (scene.camera.is_perspective) {
				   
                    ray.origin=scene.camera.position; 
					
					double x,y;
				    x= (double)rand()/RAND_MAX*2.0-1.0;
				    y= (double)rand()/RAND_MAX*2.0-1.0;

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
			// TODO
			auto parallelogram = std::make_shared<Parallelogram>();
			parallelogram->origin = read_vec3(entry["Origin"]);
            parallelogram->u = read_vec3(entry["u"]);
            parallelogram->v = read_vec3(entry["v"]);
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
