// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

#include <gif.h>
#include <fstream>

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

using namespace std;
using namespace Eigen;

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

UniformAttributes load_uniform(const std::string &filename, std::string &bunny_path){
	UniformAttributes uniform;
	json data;
	std::ifstream in(filename);
	in >> data;

	auto read_vec3f = [] (const json &x) {
		return Vector3f(x[0], x[1], x[2]);
	};

	auto read_vec4f = [] (const json &x) {
		return Vector4f(x[0], x[1], x[2], x[3]);
	};
	
	auto read_vec3d = [] (const json &x) {
		return Vector3d(x[0], x[1], x[2]);
	};
	uniform.camera.position = read_vec3d(data["Camera"]["Position"]); 
	uniform.camera.z_dir  = read_vec3d(data["Camera"]["Z_dir"]); 
	uniform.camera.y_dir  = read_vec3d(data["Camera"]["Y_dir"]);
	uniform.camera.field_of_view = data["Camera"]["FieldOfView"];
	uniform.camera.is_perspective = data["Camera"]["IsPerspective"];

	uniform.color = read_vec4f(data["Lights"]["Color"]); 
	uniform.light_source = read_vec3f(data["Lights"]["Source"]); 
	uniform.diffuse_color = read_vec3f(data["Lights"]["Diffuse"]);
	uniform.specular_color = read_vec3f(data["Lights"]["Specular"]);
	uniform.specular_exponent = data["Lights"]["Specular_exponent"];
	uniform.ambient_color = read_vec3f(data["Lights"]["Ambient_coloer"]);

	uniform.core_center.setZero();
	uniform.view.setZero().setIdentity();

	bunny_path=data["Objects"]["Path"].get<std::string>();

	return uniform;

} 

Vector3f obj_color(const UniformAttributes &uniform,const Vector4f &trans_normal,const Vector4f &trans_pos){
	Vector3f Li = (uniform.light_source - trans_pos.head(3)).normalized();
	Vector3f bisector = ((uniform.light_source - trans_pos.head(3)) + ((uniform.camera.position).cast<float> () - trans_pos.head(3))).normalized();
	Vector3f diffuse = uniform.diffuse_color * std::max(Li.dot(trans_normal.head(3)), float(0.0));
	Vector3f specular = uniform.specular_color * pow(std::max(trans_normal.head(3).dot(bisector), float(0.0)), uniform.specular_exponent);
	Vector3f color = uniform.ambient_color+ diffuse + specular;

	return color;	
} 

Vector4f compute_core_center(UniformAttributes &uniform,bool per_vertex_shading,MatrixXd &V, MatrixXi &F, MatrixXf &per_v){
	Vector3f traingle_center;
	double area = 0;
	double triangle_area;
	Vector4f core_center=uniform.core_center;
	for (unsigned i = 0; i < F.rows(); i++){
		Vector3f u, v;
		u = V.row((F(i, 0))).cast <float> () - V.row(F(i, 1)).cast <float> ();
		v = V.row((F(i, 2))).cast <float> () - V.row(F(i, 1)).cast <float> ();
		triangle_area = 0.5*(u.cross(v).norm());
		traingle_center = (1/3.0)*(V.row((F(i, 0))).cast<float>() + V.row(F(i, 1)).cast<float>() + V.row(F(i, 1)).cast<float>());
		core_center.head(3) += traingle_center*triangle_area;
		area += triangle_area;

		if (per_vertex_shading)
		{
			per_v.row(F(i, 0)) += (-u.cross(v)).normalized();
			per_v.row(F(i, 1)) += (-u.cross(v)).normalized();
			per_v.row(F(i, 2)) += (-u.cross(v)).normalized();
		}
	}

	core_center.head(3) = core_center.head(3)/area;
	core_center[3] = 1.0;

	return core_center;
}

void build_M_cam(UniformAttributes &uniform){
	Vector3d w, u, v;
	w = -uniform.camera.z_dir.normalized();
	u = (uniform.camera.y_dir.cross(w)).normalized();
	v = w.cross(u);
	Matrix4f temp;
	temp << u[0], v[0], w[0], uniform.camera.position[0],
					u[1], v[1], w[1], uniform.camera.position[1],
					u[2], v[2], w[2], uniform.camera.position[2],
					0, 0, 0, 1;

	uniform.M_cam = temp.inverse();
}

void build_bbox(UniformAttributes &uniform,MatrixXd &V){
	Vector4f l_bbox;
	Vector4f r_bbox;
	for (unsigned i = 0; i < V.cols() ; i ++){
		l_bbox[i] = V.col(i).minCoeff();
		r_bbox[i] = V.col(i).maxCoeff();
	}
	l_bbox[3] = 1; 
	r_bbox[3] = 1;

	uniform.lbn = (uniform.M_cam*l_bbox).head(3);
	uniform.rtf = (uniform.M_cam*r_bbox).head(3);
}

void build_vertices(std::string categ,vector<VertexAttributes> &v_meah,vector<VertexAttributes> &v_line,const MatrixXd &V,const MatrixXi &F,const MatrixXf &per_v){
	for (int i = 0; i < F.rows(); i++){
		for (int j = 0; j < F.cols(); j ++){
			v_meah.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
			if(categ=="wireframe"){
				if (j != 0){
					v_line.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
					v_line[v_line.size()-1].normal = per_v.row(F(i,j)).normalized();
				}
				else{
					v_line.push_back(VertexAttributes(V(F(i,j),0),V(F(i,j),1),V(F(i,j),2)));
					v_line[v_line.size()-1].normal = per_v.row(F(i,j)).normalized();
				}
			}
		}
		if (categ=="wireframe"){
			v_line.push_back(VertexAttributes(V(F(i,0),0),V(F(i,0),1),V(F(i,0),2)));
			v_line[v_line.size()-1].normal = per_v.row(F(i,0)).normalized();
		}
		else if (categ=="flat_shading"){
			Vector3f u,v;
			u = v_meah[3*i].position.head(3) - v_meah[3*i + 1].position.head(3); 
			v = v_meah[3*i + 2].position.head(3) - v_meah[3*i + 1].position.head(3);
			v_meah[3*i].normal = (-u.cross(v)).normalized(); 
			v_meah[3*i+1].normal = (-u.cross(v)).normalized(); 
			v_meah[3*i+2].normal = (-u.cross(v)).normalized(); 
		}
		else if (categ=="per_vertex_shading"){
			v_meah[3*i].normal = per_v.row(F(i,0)).normalized(); 
			v_meah[3*i+1].normal = per_v.row(F(i,1)).normalized();
			v_meah[3*i+2].normal = per_v.row(F(i, 2)).normalized();
		}
	}
}

void aspect(UniformAttributes &uniform, Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> &frameBuffer){
	float aspect_ratio = float(frameBuffer.cols())/float(frameBuffer.rows());

	double scale_y =2*uniform.camera.focal_length*tan(uniform.camera.field_of_view/2.0); 
	double scale_x =aspect_ratio*scale_y;

	if (aspect_ratio < 1)
		uniform.view(0,0) = aspect_ratio;
	else
		uniform.view(1,1) = 1/aspect_ratio; 
}


int main(int argc, char *argv[]) 
{
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
		return 1;
	} 

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(500,500);

	// Global Constants (empty in this example)
	std::string bunny_path; 
	UniformAttributes uniform=load_uniform(argv[1],bunny_path);

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		Vector4f trans_normal;
		Vector4f trans_pos;

		for (unsigned i = 0; i < 3; i++)
		{
			trans_normal[i] = va.normal[i];
			trans_pos[i] = va.position[i];
			
		}
		trans_normal[3] = 1;
		trans_pos[3] = 1;
		
		trans_normal = (uniform.core_rotate.transpose()).inverse()* trans_normal;
		trans_pos = uniform.view*uniform.core_rotate* trans_pos;

		trans_pos = uniform.M*trans_pos;
		VertexAttributes out(trans_pos[0],trans_pos[1],trans_pos[2], trans_pos[3]);
		
		out.normal = trans_normal.head(3);
		out.color.head(3) = obj_color(uniform,trans_normal,trans_pos);
		out.color[3] = uniform.color(3);
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		FragmentAttributes out(va.color(0),va.color(1),va.color(2),uniform.color(3));
		out.depth = va.position[2];
		return out;
		return FragmentAttributes(1,0,0);
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		if (fa.depth < previous.depth){
			FrameBufferAttributes out(fa.color[0]*255, fa.color[1]*255, fa.color[2]*255, fa.color[3]*255);
			out.depth = fa.depth;
			return out;
		}
		else{
			return previous;
		}
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};
	////////////////////////////////////////////////////////////////////////////////////////////////////
	bool wireframe =false;
	bool flat_shading = true;
	bool per_vertex_shading = false;
	bool render_gif =false;
	std::string categ;
	if(wireframe){
		categ="wireframe";
	}
	else if(flat_shading){
		categ="flat_shading";
	}
	else if(per_vertex_shading){
		categ="per_vertex_shading";
	}
	MatrixXd V;
	MatrixXi F;
    load_off(bunny_path, V, F);
	MatrixXf per_v;
	per_v.resize(V.rows(), V.cols());
	per_v.setZero();

	Vector4f core_center=compute_core_center(uniform,per_vertex_shading,V,F,per_v);

	build_M_cam(uniform);
    build_bbox(uniform,V);

	uniform.M_orth << 2/(uniform.rtf(0) - uniform.lbn(0)), 0, 0, -(uniform.rtf(0) + uniform.lbn(0))/(uniform.rtf(0) - uniform.lbn(0)),
				      0, 2/(uniform.rtf(1) - uniform.lbn(1)), 0, -(uniform.rtf(1) + uniform.lbn(1))/(uniform.rtf(1) - uniform.lbn(1)),
				      0, 0, 2/(uniform.lbn(2) - uniform.rtf(2)), -(uniform.rtf(2) + uniform.lbn(2))/(uniform.lbn(2) - uniform.rtf(2)),
				      0, 0, 0, 1;
	
	if (uniform.camera.is_perspective)
	{
		double tang=tan(uniform.camera.field_of_view/2);
		uniform.rtf(1) = std::abs(uniform.lbn(0))*tang;
		uniform.rtf(0) = (frameBuffer.cols()/frameBuffer.rows())*uniform.rtf(1);

		uniform.P << uniform.lbn(2), 0, 0, 0,
					0, uniform.lbn(2), 0, 0,
					0, 0, uniform.lbn(2) + uniform.rtf(2), -uniform.lbn(2) * uniform.rtf(2), 
					0, 0, 1, 0;
		uniform.M_orth = uniform.M_orth*uniform.P;
	}
	
	uniform.M = uniform.M_orth*uniform.M_cam;
    vector<VertexAttributes> v_meah;
	vector<VertexAttributes> v_line;
	aspect(uniform,frameBuffer);

	if (render_gif){
		MatrixXf trans = MatrixXf::Identity(4, 4);
		MatrixXf trans_minus = MatrixXf::Identity(4, 4);
		MatrixXf rot = MatrixXf::Identity(4, 4);
		Vector3f translate;
		translate.setZero();
		vector<uint8_t> image;
		int delay = 10;
		GifWriter g;
		if(wireframe){
			const char *fileName = "wireframe.gif";
			GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

		}
		else if(flat_shading){
			const char *fileName = "flat_shading.gif";
			GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

		}
		else if(per_vertex_shading){
			const char *fileName = "per_vertex_shading.gif";
			GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);
		}

		for (float i = 0; i < 1; i += 0.05)
		{
			double theta = (360*i / 180.0) * M_PI;
			rot(0, 0) = cos(theta);
			rot(0, 1) = -sin(theta);
			rot(1, 1) = cos(theta);
			rot(1, 0) = sin(theta);
			trans.col(3) = core_center;
			trans_minus.col(3) = -core_center;
			trans_minus(3,3) = 1.0;
			uniform.core_rotate = trans * rot * trans_minus;	
			translate[1] +=0.002;
			trans.col(3).head(3) = translate;
			uniform.core_rotate = trans * uniform.core_rotate;

			frameBuffer.setConstant(FrameBufferAttributes());
			build_vertices(categ,v_meah,v_line,V,F,per_v);
			if (wireframe)
			{
				rasterize_lines(program, uniform, v_line, 0.5, frameBuffer);
			}
			else if (flat_shading || per_vertex_shading)
			{
				rasterize_triangles(program, uniform, v_meah, frameBuffer);
			}
			framebuffer_to_uint8(frameBuffer, image);
			GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
		}
		GifEnd(&g);	
	}
	else{
		uniform.core_rotate = MatrixXf::Identity(4, 4);
		frameBuffer.setConstant(FrameBufferAttributes());
		build_vertices(categ,v_meah,v_line,V,F,per_v);
		vector<uint8_t> image;
		if (wireframe){
			const char *fileName = "wireframe.png";
			rasterize_lines(program, uniform, v_line, 0.5, frameBuffer);
			framebuffer_to_uint8(frameBuffer,image);
		stbi_write_png(fileName, frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
		}
		else if (flat_shading)
		{
			const char *fileName = "flat_shading.png";
			rasterize_triangles(program, uniform, v_meah, frameBuffer);
			framebuffer_to_uint8(frameBuffer,image);
		stbi_write_png(fileName, frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
		}
		else if(per_vertex_shading){
			const char *fileName = "per_vertex_shading.png";
			rasterize_triangles(program, uniform, v_meah, frameBuffer);
			framebuffer_to_uint8(frameBuffer,image);
		stbi_write_png(fileName, frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows()*4);
		}
	}
	
	return 0;
	
}