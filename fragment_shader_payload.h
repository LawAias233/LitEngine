#pragma once

#include <Eigen/Dense>

#include "Texture.h"

class fragment_shader_payload
{
public :
	Eigen::Vector3f view_pos;
	Eigen::Vector3f color;
	Eigen::Vector3f normal;
	Eigen::Vector2f texture_coords;

	std::shared_ptr<Texture>texture;

public:
	fragment_shader_payload(Eigen::Vector3f& v, Eigen::Vector3f& c, Eigen::Vector3f& n, Eigen::Vector2f& tc, std::shared_ptr<Texture> t)
		:view_pos(v), color(c), normal(n), texture_coords(tc), texture(t) {}
};

