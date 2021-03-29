#pragma once

#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>

#include "Triangle.h"
#include "fragment_shader_payload.h"

struct Light
{
	Eigen::Vector3f lightPos;
	Eigen::Vector3f lightIntensity;
};

class Rasterizer
{
public:
	Rasterizer(int w, int h) :width(w), height(h)
	{
		frame_buf.resize(w*h);
		depth_buf.resize(w*h,100000.0f);
		lights.push_back({ {-1.0f, 0.0f, 0.0f},{27.5, 27.5f, 27.5f} });
		//lights.push_back({ {-0.5f, -0.5f, 10.0f},{7.0f, 7.0f, 7.0f} });
	}

	void set_camera(const Eigen::Matrix4f&, const Eigen::Vector3f&);
	void set_model(const Eigen::Matrix4f&);
	void set_projection(const Eigen::Matrix4f&);
	void set_texture(std::shared_ptr<Texture>);

	std::vector<Eigen::Vector3f>& get_frame_buf() 
	{
		return frame_buf; 
	}

	void rasterize(std::vector<Triangle*> triangleList);
	void rasterize_triangle(Triangle& ,const std::array<Eigen::Vector3f, 3>&);

	Eigen::Vector3f phongShader(const fragment_shader_payload&, const std::vector<Light>&);
	Eigen::Vector3f textureShader(const fragment_shader_payload&, const std::vector<Light>&);

private:
	int width, height;
	const int cmp = 3;
	std::vector<Eigen::Vector3f> frame_buf; //÷°ª∫¥Ê
	std::vector<float> depth_buf; //…Ó∂»ª∫¥Ê

	Eigen::Vector3f cameraPos;
	Eigen::Matrix4f modelMatrix;
	Eigen::Matrix4f cameraMatrix;
	Eigen::Matrix4f projectionMatrix;

	std::shared_ptr<Texture>texture;
	std::vector<Light> lights;
	//Eigen::Vector3f (*shader_pointer)(fragment_shader_payload&);
};

