// C++ include
#include <iostream>
#include <string>
#include <vector>


// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
/* #include "json.hpp"
using json = nlohmann::json; */

#include <fstream>
#include <gif.h>

using namespace Eigen;
using namespace std;

///////////////////////////////////////////////////

void uniform_initial(UniformAttributes &uniform){
	uniform.camera.position << 0, 0, 2;
	uniform.camera.gaze_direction << 0,0,1;
	uniform.camera.view_up << 0,1,0;
	uniform.camera.field_of_view =0.7854;
	uniform.camera.is_perspective = false;
	uniform.color << 16,16,16,16;
	uniform.light_source << 0, 2, 0;
	uniform.diffuse_color << 0.5, 0.5, 0.5;
	uniform.specular_color << 0.2, 0.2, 0.2;
	uniform.specular_exponent = 1.0;
	uniform.ambient_color << 0.2, 0.2, 0.2;
}

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

void build_wireframe(vector<VertexAttributes> &vertices_lines, int row, MatrixXd &V, MatrixXi &F, MatrixXf &V_p){
	vertices_lines.push_back(VertexAttributes(V(F(row,0),0),V(F(row,0),1),V(F(row,0),2)));
	vertices_lines[vertices_lines.size()-1].normal = V_p.row(F(row,0)).normalized();
}

void build_flat_shading(vector<VertexAttributes> &vertices_mesh, int row, MatrixXd &V, MatrixXi &F){
	Vector3f u,v;
	u = vertices_mesh[3*row].position.head(3) - vertices_mesh[3*row + 1].position.head(3); 
	v = vertices_mesh[3*row + 2].position.head(3) - vertices_mesh[3*row + 1].position.head(3);
	vertices_mesh[3*row].normal = (-u.cross(v)).normalized(); 
	vertices_mesh[3*row+1].normal = (-u.cross(v)).normalized(); 
	vertices_mesh[3*row+2].normal = (-u.cross(v)).normalized(); 
}

void build_per_vertex_shading(vector<VertexAttributes> &vertices_mesh, int row, MatrixXi &F, MatrixXf &V_p){
 	vertices_mesh[3*row].normal = V_p.row(F(row,0)).normalized(); 
	vertices_mesh[3*row+1].normal = V_p.row(F(row,1)).normalized();
	vertices_mesh[3*row+2].normal = V_p.row(F(row, 2)).normalized();
}

void build_M_camera(UniformAttributes &uniform){
	// computing transformation from wold to camera
	Vector3d w, u, v;
	w = -uniform.camera.gaze_direction.normalized();
	u = (uniform.camera.view_up.cross(w)).normalized();
	v = w.cross(u);

	Matrix4f tmp;
	tmp << u[0], v[0], w[0], uniform.camera.position[0],
			u[1], v[1], w[1], uniform.camera.position[1],
			u[2], v[2], w[2], uniform.camera.position[2],
			0, 0, 0, 1;

	uniform.M_cam = tmp.inverse();
}

void camera_to_canonocal(UniformAttributes &uniform,MatrixXd V){
	Vector4f lbn_world, rtf_world;
	for (unsigned i = 0; i < V.cols() ; i ++){
		lbn_world[i] = V.col(i).minCoeff();
		rtf_world[i] = V.col(i).maxCoeff();
		
	}
	lbn_world[3] = 1; 
	rtf_world[3] = 1;
	//  making the box slightly bigger than the bounding box so that 
	// the bunny does not fill up the entire space
	lbn_world(0) -= 0.1;
	lbn_world(1) -= 0.1;
	lbn_world(2) -= 0.1;
	
	rtf_world(0) += 0.1;
	rtf_world(1) += 0.1;
	rtf_world(2) += 0.1;
	// tranforming from world to camera frame
	uniform.lbn = (uniform.M_cam*lbn_world).head(3);
	uniform.rtf = (uniform.M_cam*rtf_world).head(3);
}

void build_M_orth(UniformAttributes &uniform){
	uniform.M_orth << 2/(uniform.rtf(0) - uniform.lbn(0)), 0, 0, -(uniform.rtf(0) + uniform.lbn(0))/(uniform.rtf(0) - uniform.lbn(0)),
				0, 2/(uniform.rtf(1) - uniform.lbn(1)), 0, -(uniform.rtf(1) + uniform.lbn(1))/(uniform.rtf(1) - uniform.lbn(1)),
				0, 0, 2/(uniform.lbn(2) - uniform.rtf(2)), -(uniform.rtf(2) + uniform.lbn(2))/(uniform.lbn(2) - uniform.rtf(2)),
				0, 0, 0, 1;
}





int main() 
{

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	UniformAttributes uniform;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		Vector4f transformed_normal, transformed_vector;

		for (unsigned i = 0; i < 3; i++)
		{
			transformed_vector[i] = va.position[i];
			transformed_normal[i] = va.normal[i];
		}
		transformed_vector[3] = 1;
		transformed_normal[3] = 1;
		transformed_vector = uniform.view*uniform.core_rotate* transformed_vector;

		transformed_normal = (uniform.core_rotate.transpose()).inverse()* transformed_normal;
		Vector3f Li = (uniform.light_source - transformed_vector.head(3)).normalized();
		Vector3f bisector = ((uniform.light_source - transformed_vector.head(3)) + ((uniform.camera.position).cast<float> () - transformed_vector.head(3))).normalized();
		Vector3f diffuse, specular, color;
		diffuse = uniform.diffuse_color * std::max(Li.dot(transformed_normal.head(3)), float(0.0));
		specular = uniform.specular_color * pow(std::max(transformed_normal.head(3).dot(bisector), float(0.0)), uniform.specular_exponent);

		color = uniform.ambient_color + diffuse + specular;
		transformed_vector = uniform.M*transformed_vector;
		VertexAttributes out(transformed_vector[0],transformed_vector[1],transformed_vector[2], transformed_vector[3]);
		out.normal = transformed_normal.head(3);
		out.color.head(3) = color;
		out.color[3] = uniform.color(3);
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		
		FragmentAttributes out(va.color(0),va.color(1),va.color(2),uniform.color(3));

		out.depth = va.position[2];
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		if (fa.depth >= previous.depth){
			return previous;
		}
		else{
			FrameBufferAttributes out(fa.color[0]*255, fa.color[1]*255, fa.color[2]*255, fa.color[3]*255);
			out.depth = fa.depth;
			return out;
		}
	};

	bool wireframe= false; 
	bool flat_shading= true;
	bool per_vertex_shading= false;
	bool render_gif=true; 
	
	uniform_initial(uniform);
	

	// loading the mesh
	MatrixXd V;
	MatrixXi F;
	std::string filename = "../data/bunny.off" ;
	MatrixXf V_p;

	Vector3f traingle_center;
	double area = 0, triangle_area;
	uniform.core_center.setZero();

	load_off(filename, V, F);

	V_p.resize(V.rows(), V.cols());
	V_p.setZero();
	for (unsigned i = 0; i < F.rows(); i++){
		Vector3f u, v;
		// computing bary center of object
		u = V.row((F(i, 0))).cast <float> () - V.row(F(i, 1)).cast <float> ();
		v = V.row((F(i, 2))).cast <float> () - V.row(F(i, 1)).cast <float> ();
		triangle_area = 0.5*(u.cross(v).norm());
		traingle_center = (1/3.0)*(V.row((F(i, 0))).cast<float>() + V.row(F(i, 1)).cast<float>() + V.row(F(i, 1)).cast<float>());
		uniform.core_center.head(3) += traingle_center*triangle_area;
		area += triangle_area;

		if (per_vertex_shading)
		// computing average normal at each vertex for per vertex shading
		// normalising of the entire normal is done at line 194
		{
			V_p.row(F(i, 0)) += (-u.cross(v)).normalized();
			V_p.row(F(i, 1)) += (-u.cross(v)).normalized();
			V_p.row(F(i, 2)) += (-u.cross(v)).normalized();
		}
	}

	uniform.core_center.head(3) = uniform.core_center.head(3)/area;
	uniform.core_center[3] = 1.0;

	// pushing triangle information into vertices
	vector<VertexAttributes> vertices_mesh;
	// pushing line information into vertices
	vector<VertexAttributes> vertices_lines;
	for (unsigned i = 0; i < F.rows(); i++){
		for (unsigned j = 0; j < F.cols(); j ++){
			vertices_mesh.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
			if(wireframe){
				if (j == 0){
					vertices_lines.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
					// setting normals to zero since a normal to a line is not defined os ligthing is constant
					vertices_lines[vertices_lines.size()-1].normal = V_p.row(F(i,j)).normalized();
				}
				else{
					vertices_lines.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
					// setting normals to zero since a normal to a line is not defined os ligthing is constant
					vertices_lines[vertices_lines.size()-1].normal = V_p.row(F(i,j)).normalized();
					vertices_lines.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
					// setting normals to zero since a normal to a line is not defined os ligthing is constant
					vertices_lines[vertices_lines.size()-1].normal = V_p.row(F(i,j)).normalized();
				}
			}
		}
		if (wireframe){
			build_wireframe(vertices_lines,i,V,F,V_p);
		}
		if (flat_shading){
			// computing the face normals
			build_flat_shading(vertices_mesh,i,V,F);
		}
		if (per_vertex_shading){
			// normalizing the normals at each vertex 
			build_per_vertex_shading(vertices_lines,i,F,V_p);
		}
	}

	// computing the transformation matrices
	build_M_camera(uniform);
	

	// compututing transformation from camera view to canonical view volume
	camera_to_canonocal(uniform,V);
	
	// computing M_orth
	// not doing (n - f) here since, the bounded box limits are already transformed to the right axis before while
	// transforming the bounding box from the world frame to the camera frame
	if (uniform.camera.is_perspective)
	{
		// to do for perspective
		uniform.rtf(1) = std::abs(uniform.lbn(0))*tan(uniform.camera.field_of_view/2);
		uniform.rtf(0) = (1.0*frameBuffer.cols()/frameBuffer.rows())*uniform.rtf(1);
		// uniform.rtf(2) = -2*uniform.lbn(2);
		uniform.P << uniform.lbn(2), 0, 0, 0,
			0, uniform.lbn(2), 0, 0,
			0, 0, uniform.lbn(2) + uniform.rtf(2), -uniform.lbn(2) * uniform.rtf(2), 
			0, 0, 1, 0;
		build_M_orth(uniform);
		uniform.M_orth = uniform.M_orth*uniform.P;
	}
	else{
		build_M_orth(uniform);
	}
	
	// M_vp is not computed as it is carried out in the rasterize triangle part
	// M_object to world is assumed to be identity 
	uniform.M = uniform.M_orth*uniform.M_cam;
	// storing the inverse to tranform normals computed at each vertex into the canonical view volume space
	Vector4f camera_location;
	camera_location << uniform.light_source(0), uniform.light_source(1), uniform.light_source(2), 1;


	// Add a transformation to compensate for the aspect ratio of the framebuffer
	float aspect_ratio = float(frameBuffer.cols())/float(frameBuffer.rows());

	uniform.view <<
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1;

	if (aspect_ratio < 1)
		uniform.view(0,0) = aspect_ratio;
	else
		uniform.view(1,1) = 1/aspect_ratio;


	if (render_gif){
		MatrixXf trans = MatrixXf::Identity(4, 4);
		MatrixXf trans_minus = MatrixXf::Identity(4, 4);
		MatrixXf rot = MatrixXf::Identity(4, 4);
		Vector3f translate;
		translate.setZero();
		const char *fileName = "bunny.gif";
		vector<uint8_t> image;
		int delay = 10;
		GifWriter g;
		GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

		for (float i = 0; i < 1; i += 0.03)
		{
			double theta = (360*i / 180.0) * M_PI;
			trans.col(3) = uniform.core_center;
			trans_minus.col(3) = -uniform.core_center;
			trans_minus(3,3) = 1.0;
			rot(0, 0) = cos(theta);
			rot(0, 2) = sin(theta);
			rot(2, 2) = cos(theta);
			rot(2, 0) = -sin(theta);

			uniform.core_rotate = trans * rot * trans_minus;
			// translates object after rotation
			translate[0] += -0.001;	
			translate[1] += -0.001; 

			trans.col(3).head(3) = translate;
			uniform.core_rotate = trans * uniform.core_rotate;

			frameBuffer.setConstant(FrameBufferAttributes());
			if (flat_shading||per_vertex_shading)
			{
				rasterize_triangles(program, uniform, vertices_mesh, frameBuffer);
			}
			if (wireframe)
			{
				rasterize_lines(program, uniform, vertices_lines, 0.5, frameBuffer);
			}

			framebuffer_to_uint8(frameBuffer, image);
			GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
		}

		GifEnd(&g);
		return 0;	
		
	}
	else{
		uniform.core_rotate = MatrixXf::Identity(4, 4);
		frameBuffer.setConstant(FrameBufferAttributes());
		if (flat_shading ||per_vertex_shading)
		{
			rasterize_triangles(program, uniform, vertices_mesh, frameBuffer);
		}
		if (wireframe){
			rasterize_lines(program, uniform, vertices_lines, 0.5, frameBuffer);
		}
		
		
	}
	vector<uint8_t> image;
	framebuffer_to_uint8(frameBuffer,image);
	stbi_write_png("bunny.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);

	return 0;	

}
