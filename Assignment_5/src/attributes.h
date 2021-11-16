#pragma once

#include <Eigen/Core>

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
        r.position = alpha*a.position + beta*b.position + gamma*c.position;
		// not modifing the normal since it is used to compute lighting in the world frame
		r.normal =  alpha*a.normal + beta*b.normal + gamma*c.normal;//xiugai
		r.color = alpha * a.color + beta * b.color + gamma * c.color;//xiugai
        return r;
    }

	Eigen::Vector4f position;
	Eigen::Vector3f normal;//xiugai
	Eigen::Vector4f color;//xiugai
};

class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4f color;
	float depth;//xiugai
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
	}

	Eigen::Matrix<uint8_t,4,1> color;
	float depth=2;
};

class UniformAttributes
{
	////////////////////////////////////////////////////////////////////////////////
	// Define types & classes
	////////////////////////////////////////////////////////////////////////////////
	struct Camera {
		bool is_perspective;
		Eigen::Vector3d position;
		double field_of_view; // between 0 and PI
		double focal_length;
		double lens_radius; // for depth of field
		Eigen::Vector3d gaze_direction;
		Eigen::Vector3d view_up;
	};

	public:
	Camera camera;
	Eigen::Vector4f core_center; 
	Eigen::MatrixXf core_rotate;
	Eigen::MatrixXf view;

	
	Eigen::Matrix4f M_orth, M_cam, M_model, M, M_inv, M_orth_inv, M_cam_inv;
	Eigen::Vector3f lbn, rtf; // lower and upper limit of camera view
	Eigen::Vector4f color;
	Eigen::Vector3f light_source;
	Eigen::Matrix4f P;

	Eigen::Vector3f diffuse_color, specular_color, ambient_color;
	float specular_exponent;


};