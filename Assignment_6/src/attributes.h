#pragma once

#include <Eigen/Core>
#include <utility>


#include <vector>

class VertexAttributes
{
	public:
	VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
	{
		position << x,y,z,w;
		color << 1,0,0,1;
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
		r.color = alpha*a.color + beta*b.color + gamma*c.color;
        return r;
    }

    friend std::ostream &operator<<(std::ostream &os, const VertexAttributes &v);

    Eigen::Vector4f position;
	Eigen::Vector4f color{0,0,0,1};
};

class FragmentAttributes
{
	public:
	FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1)
	{
		color << r,g,b,a;
	}

	Eigen::Vector4f color;
};

class FrameBufferAttributes
{
	public:
	FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
	{
		color << r,g,b,a;
	}

	Eigen::Matrix<uint8_t,4,1> color;
};

class UniformAttributes
{
	public:
    int width = 0;
    int height = 0;
    int zoomIn = 0;
    float moveLeft = 0;
    float moveUp = 0;
    Eigen::Matrix<float,4,4> view_Matri;
    Eigen::Matrix<float,4,4> inver_Matri;
    UniformAttributes();
};

class Triangle {

public:
    VertexAttributes vs[3];
    Triangle(const VertexAttributes &v1, const VertexAttributes &v2, const VertexAttributes &v3);
    bool isInside(Eigen::Vector4f &vertex);
    void shift(Eigen::Vector4f &shift);
    void shift(float x, float y);
    void rotate(float degree);
    void scale(float factor);
    Eigen::Vector4f centroid();
    bool highlight = false;

private:
    static float sign (Eigen::Vector4f &p1, Eigen::Vector4f &p2, Eigen::Vector4f &p3);
};

