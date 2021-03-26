#pragma once
#include "Triangle.h"
#include <vector>
#include <memory>
#include <Eigen/Dense>

class Rasterizer
{
public:
	Rasterizer(int w, int h) :width(w), height(h)
	{
		frame_buf.resize(w*h);
		depth_buf.resize(w*h,100000.0f);
	}

	void set_camera(Eigen::Matrix4f);
	void set_model(Eigen::Matrix4f);
	void set_projection(Eigen::Matrix4f);

	std::vector<Eigen::Vector3f>& get_frame_buf() 
	{
		return frame_buf; 
	}

	void rasterize(Triangle* te);

	void rasterize(std::vector<Triangle*> triangleList);
	void rasterize_triangle(Triangle&);

private:
	int width, height;
	const int cmp = 3;
	std::vector<Eigen::Vector3f> frame_buf; //֡����
	std::vector<float> depth_buf; //��Ȼ���

	Eigen::Matrix4f modelMatrix;
	Eigen::Matrix4f cameraMatrix;
	Eigen::Matrix4f projectionMatrix;
};

