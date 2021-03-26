#include "Triangle.h"

Triangle::Triangle()
{
	//vertexs.resize(3);
	vertexs[0] << 0.0f, 0.0f, 0.0f, 1.0f;
	vertexs[1] << 0.0f, 0.0f, 0.0f, 1.0f;
	vertexs[2] << 0.0f, 0.0f, 0.0f, 1.0f;

	//colors.resize(3);
	colors[0] << 0.0f, 0.0f, 0.0f;
	colors[1] << 0.0f, 0.0f, 0.0f;
	colors[2] << 0.0f, 0.0f, 0.0f;

	//normals.resize(3);
	//tex_coords.resize(3);
}


void Triangle::set_vertex(int ind, Eigen::Vector4f v)
{
	this->vertexs[ind] = v;
}

void Triangle::set_color(int ind, float r, float g, float b)
{
	this->colors[ind] = Eigen::Vector3f((float)r/255.99f,(float)g/255.99f,(float)b/255.99f);
}

void Triangle::set_normal(int ind, Eigen::Vector3f n)
{
	this->normals[ind] = n;
}

void Triangle::set_texCoord(int ind, Eigen::Vector2f t)
{
	this->tex_coords[ind] = t;
}

std::array<Eigen::Vector4f, 3> Triangle::toVector4() const
{
	std::array<Eigen::Vector4f, 3>res;
	std::transform(&vertexs[0], &vertexs[2]+1, res.begin()
		, [](auto& v) {return Eigen::Vector4f(v.x(), v.y(), v.z(), 1.0f); });
	return res;
}

bool Triangle::getBarycentricCoords3d(Eigen::Vector3f p, float f[3])
{
	//使用重心坐标法，计算p在t中的重心坐标
	//出于方便，总是抛弃z轴方向，将三角形和点都投影到xy平面中
	//由0 1 2三点组成的三角形面积为S则有 2*s = (0_y - 2_y)(1_x-2_x)+(1_y-2_y)(2_x-0_x) = v1*u1+v2*u2;
	//由p 1 2三点组成的三角形面积为S则有 2*s = (p_y - 2_y)(1_x-2_x)+(1_y-2_y)(2_x-p_x) = v3*u1+v2*u3;
	//由0 p 2三点组成的三角形面积为S则有 2*s = (0_y - 2_y)(p_x-2_x)+(p_y-2_y)(2_x-0_x) = v1*u4+v3*u2;
	//于是通过面积之比解出重心坐标
	auto& v = vertexs;
	float v1 = v[0].y() - v[2].y();
	float v2 = v[1].y() - v[2].y();
	float v3 = p.y() - v[2].y();
	float v4 = p.x() - v[2].x();
	float u1 = v[1].x() - v[2].x();
	float u2 = v[2].x() - v[0].x();
	float u3 = v[2].x() - p.x();
	float u4 = p.x() - v[2].x();
	float oneOverDemon = 1 / (v1 * u1 + v2 * u2);
	f[0] = (v3 * u1 + v2 * u3) * oneOverDemon;
	f[1] = (v1 * u4 + v3 * u2) * oneOverDemon;
	f[2] = 1.0f - f[0] - f[1];
	return true;
}