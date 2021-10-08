// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"


// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere() {
	std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

	const std::string filename("sphere_orthographic.png");
	MatrixXd C = MatrixXd::Zero(200,200); // Store the color
	MatrixXd A = MatrixXd::Zero(200,200); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);

}

void raytrace_parallelogram() {
	std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

	const std::string filename("plane_orthographic.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(0.0,0.0,0.0);
	Vector3d pgram_u(-0.5,0.5, 0.0);
	Vector3d pgram_v(-0.5, 0.0, 0.0);

	// Single light source
	const Vector3d light_position(-1,1,1);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// TODO: Check if the ray intersects with the parallelogram
			Matrix3d leftEqu;
			Vector3d rightEqu;
			Vector3d uvt;
			for(int i=0;i<3;i++){
				leftEqu(i,0)=pgram_origin[i]-pgram_u[i];//i=0,(a-b)x; i=1,(a-b)y; i=2,(a-b)z
				leftEqu(i,1)=pgram_origin[i]-pgram_v[i];//i=0,(a-c)x; i=1,(a-c)y; i=2,(a-c)z
				leftEqu(i,2)=ray_direction[i];
			}
			rightEqu= pgram_origin-ray_origin;
			uvt= leftEqu.colPivHouseholderQr().solve(rightEqu);

			if (uvt[0]<=1 &&uvt[0]>=0 &&uvt[1]<=1&&uvt[1]>=0&&uvt[2]>0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection;
				ray_intersection = uvt[2] * ray_direction + ray_origin;
				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = pgram_u.cross(pgram_v).normalized(); //(b-a)x(c-a)/|(b-a)x(c-a)|

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_perspective() {
	std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

	const std::string filename("plane_perspective.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);
	Vector3d camera(0,0,1);

	// TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
	Vector3d pgram_origin(-0.5,-0.1,-0.4);
	Vector3d pgram_u(-0.4,0.5,-0.4);
	Vector3d pgram_v(-0.1,0.0,-0.4);
	// Sphere
	Vector3d sphere_ori(1,0.5,-0.5);
	const double radius=0.3;

	// Single light source
	const Vector3d light_position(-1,1,1);
	const double screen=-1.0;

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// TODO: Prepare the ray (origin point and direction)
			Vector3d ray_origin = camera;
			Vector3d p_direction = origin + double(i)*x_displacement + double(j)*y_displacement;
            p_direction[2]=p_direction[2]+screen;
			Vector3d ray_direction= p_direction-ray_origin;
			// TODO: Check if the ray intersects with the parallelogram
			Matrix3d leftEqu;
			Vector3d rightEqu;
			Vector3d uvt;
			for(int i=0;i<3;i++){
				leftEqu(i,0)=pgram_origin[i]-pgram_u[i];//i=0,(a-b)x; i=1,(a-b)y; i=2,(a-b)z
				leftEqu(i,1)=pgram_origin[i]-pgram_v[i];//i=0,(a-c)x; i=1,(a-c)y; i=2,(a-c)z
				leftEqu(i,2)=ray_direction[i];
			}
			rightEqu= pgram_origin-ray_origin;
			uvt= leftEqu.colPivHouseholderQr().solve(rightEqu);

			if (uvt[0]<=1 &&uvt[0]>=0 &&uvt[1]<=1&&uvt[1]>=0&&uvt[2]>0) {
				// TODO: The ray hit the parallelogram, compute the exact intersection point
				Vector3d ray_intersection;
				ray_intersection = uvt[2] * ray_direction + ray_origin;
				// TODO: Compute normal at the intersection point
				Vector3d ray_normal = pgram_u.cross(pgram_v).normalized();//(b-a)x(c-a)/|(b-a)x(c-a)|

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}

			//Sphere perspective 
			double a=ray_direction.dot(ray_direction); //d*d
			double b=2*ray_direction.dot(ray_origin-sphere_ori); //2d(e-c)
			double c=(ray_origin-sphere_ori).dot(ray_origin-sphere_ori)-radius*radius;// (e-c)(e-c)-R*R
			if(b*b-4*a*c>=0){
				Vector3d ray_intersection;
				double t,t1,t2;
				if(b*b-4*a*c==0){
					t=(-b)/(2*a);
				}
				else{
					t1=(-b+sqrt(b*b-4*a*c))/(2*a);
					t2=(-b-sqrt(b*b-4*a*c))/(2*a);
					if(t1>=0 && t2>=0){
						if(t1<t2) t=t1;
						else t=t2;
					}
					else if(t1>=0 && t2<0) t=t1;
					else t=t2;
				}
				ray_intersection=ray_origin+t * ray_direction;
				Vector3d ray_normal = (ray_intersection - sphere_ori).normalized();//p-c

				// Simple diffuse model
				C(i,j) = (light_position-ray_intersection).normalized().transpose()*ray_normal;

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_shading(){
	std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

	const std::string filename("shading.png");
	MatrixXd C = MatrixXd::Zero(800,800); // Store the color
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask

	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/C.cols(),0,0);
	Vector3d y_displacement(0,-2.0/C.rows(),0);

	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	for (unsigned i=0; i < C.cols(); ++i) {
		for (unsigned j=0; j < C.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = origin + double(i)*x_displacement + double(j)*y_displacement;
			Vector3d ray_direction = RowVector3d(0,0,-1);

			// Intersect with the sphere
			// NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
			Vector2d ray_on_xy(ray_origin(0),ray_origin(1));
			const double sphere_radius = 0.9;

			if (ray_on_xy.norm() < sphere_radius) {
				// The ray hit the sphere, compute the exact intersection point
				Vector3d ray_intersection(ray_on_xy(0),ray_on_xy(1),sqrt(sphere_radius*sphere_radius - ray_on_xy.squaredNorm()));

				// Compute normal at the intersection point
				Vector3d ray_normal = ray_intersection.normalized();

				// TODO: Add shading parameter here
				diffuse(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				specular(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;

				// Simple diffuse model
				C(i,j) = ambient + diffuse(i,j) + specular(i,j);

				// Clamp to zero
				C(i,j) = std::max(C(i,j),0.);

				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(C,C,C,A,filename);
}

void raytrace_rgb_shading(){
	std::cout << "Simple ray tracer, one sphere with different RGB shading" << std::endl;

	const std::string filename("shading_rgb.png");
	MatrixXd A = MatrixXd::Zero(800,800); // Store the alpha mask
	
	MatrixXd R = MatrixXd::Zero(800,800); 
	MatrixXd G = MatrixXd::Zero(800,800); 
	MatrixXd B = MatrixXd::Zero(800,800); 

	
	// The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
	Vector3d origin(-1,1,1);
	Vector3d x_displacement(2.0/R.cols(),0,0);
	Vector3d y_displacement(0,-2.0/R.rows(),0);
	Vector3d camera(0,0,1);


	// Single light source
	const Vector3d light_position(-1,1,1);
	double ambient = 0.1;
	MatrixXd diffuse = MatrixXd::Zero(800, 800);
	MatrixXd specular = MatrixXd::Zero(800, 800);

	const double screen=-1.0;

	// Sphere
	Vector3d sphere_ori(0.0,0.5,-2);
	const double radius=1;


	for (unsigned i=0; i < R.cols(); ++i) {
		for (unsigned j=0; j < R.rows(); ++j) {
			// Prepare the ray
			Vector3d ray_origin = camera;
			Vector3d p_direction = origin + double(i)*x_displacement + double(j)*y_displacement;
            p_direction[2]=p_direction[2]+screen;
			Vector3d ray_direction= p_direction-ray_origin;

			//Sphere perspective 
			double a=ray_direction.dot(ray_direction); //d*d
			double b=2*ray_direction.dot(ray_origin-sphere_ori); //2d(e-c)
			double c=(ray_origin-sphere_ori).dot(ray_origin-sphere_ori)-radius*radius;// (e-c)(e-c)-R*R

			if (b*b-4*a*c>=0) {
				Vector3d ray_intersection;
				double t,t1,t2;
				if(b*b-4*a*c==0){
					t=(-b)/(2*a);
				}
				else{
					t1=(-b+sqrt(b*b-4*a*c))/(2*a);
					t2=(-b-sqrt(b*b-4*a*c))/(2*a);
					if(t1>=0 && t2>=0){
						if(t1<t2) t=t1;
						else t=t2;
					}
					else if(t1>=0 && t2<0) t=t1;
					else t=t2;
				}
				// The ray hit the sphere, compute the exact intersection point
				ray_intersection=ray_origin+t * ray_direction;

				// Compute normal at the intersection point
				Vector3d ray_normal = (ray_intersection - sphere_ori).normalized();//p-c

				// TODO: Add shading parameter here
				double phong=100;
				double diff_coe=1;
				double spec_coe=4;
				diffuse(i,j) = (light_position-ray_intersection).normalized().transpose() * ray_normal;
				specular(i,j) = pow(((ray_origin - ray_intersection) + (light_position - ray_intersection)).normalized().transpose() * ray_normal,phong);
				
				// Simple diffuse model
				R(i,j) = ambient + diff_coe*diffuse(i,j) + spec_coe*specular(i,j);
				G(i,j) = ambient + diff_coe*diffuse(i,j) + spec_coe*specular(i,j);
				B(i,j) = ambient + 0.6*diffuse(i,j) + 1.5*specular(i,j);

				// Clamp to zero
				R(i,j) = std::max(R(i,j),0.);
				G(i,j) = std::max(G(i,j),0.);
				B(i,j) = std::max(B(i,j),0.);


				// Disable the alpha mask for this pixel
				A(i,j) = 1;
			}
		}
	}

	// Save to png
	write_matrix_to_png(R,G,B,A,filename);
}

int main() {
	raytrace_sphere();
	raytrace_parallelogram();
	raytrace_perspective();
	raytrace_shading();
	raytrace_rgb_shading();

	return 0;
}
