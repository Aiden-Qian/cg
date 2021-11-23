#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
	}

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes& a,
        const VertexAttributes& b,
        const VertexAttributes& c,
        const float alpha, 
        const float beta, 
        const float gamma
    ) 
    {
        VertexAttributes r;
        r.position = alpha*a.position/a.position[3] + beta*b.position/b.position[3] + gamma*c.position/c.position[3];
		// not modifing the normal since it is used to compute lighting in the world frame
		r.normal =  alpha*a.normal + beta*b.normal + gamma*c.normal;
		r.color = alpha * a.color + beta * b.color + gamma * c.color;

		return r;
    }

	Eigen::Vector4f position;
	Eigen::Vector3f normal;
	Eigen::Vector4f color;
};

class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}
	float depth;
	Eigen::Vector4f color;
	
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
	}
	float depth = 4;
	Eigen::Matrix<uint8_t,4,1> color;
	
};

class UniformAttributes
{
	struct Camera {
		bool is_perspective;
		Eigen::Vector3d position;
		double field_of_view;
		double focal_length;
		double lens_radius; 
		Eigen::Vector3d z_dir;
		Eigen::Vector3d y_dir;
	};
	public:
		Camera camera;	
		Eigen::MatrixXf core_rotate; 
		Eigen::Matrix4f view;
		Eigen::Matrix4f M_orth, M_cam, M,P;

		Eigen::Vector3f lbn, rtf,light_source;
		Eigen::Vector3f diffuse_color, specular_color, ambient_color;
		Eigen::Vector4f core_center,color;

		float specular_exponent;
};
		
		