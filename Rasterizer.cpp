#include "Rasterizer.h"
#include <algorithm>
#include <iostream>

static float interpolate_f(float alpha, float beta, float gamma, const float vert1, const float vert2, const float vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
	auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
	auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

	u /= weight;
	v /= weight;

	return Eigen::Vector2f(u, v);
}


void Rasterizer::rasterize(std::vector<Triangle*>triangleList)
{

	for (const auto& te : triangleList)
	{
		Triangle t(*te);
		Eigen::Vector4f v[3];
		//1.进行MC变换
		Eigen::Matrix4f mcp =projectionMatrix * cameraMatrix * modelMatrix;
		//Eigen::Matrix4f mcp =cameraMatrix * modelMatrix;
		//Eigen::Matrix4f mcp =projectionMatrix * cameraMatrix * modelMatrix;
		//Eigen::Matrix4f mcp =projectionMatrix * cameraMatrix * modelMatrix;
		for (int i = 0; i < 3; ++i) v[i] = mcp*t.vertexs[i];
		//2.透视除法
		for (int i = 0; i < 3; ++i)
		{
			v[i].x() /= v[i].w();
			v[i].y() /= v[i].w();
			v[i].z() /= v[i].w();
		}
		//3.法向量修正
		//4.视口变换
		for (int i = 0; i < 3; ++i)
		{
			v[i].x() = (v[i].x() + 1.0f) * width * 0.5f;
			v[i].y() = (v[i].y() + 1.0f) * height * 0.5f;
			v[i].z() = 50.0f * v[i].z() + 50.0f;//扩大差异
			t.set_vertex(i, v[i]);
		}
		//5.渲染最终的三角形
		rasterize_triangle(t);
	}
}

void Rasterizer::rasterize_triangle(Triangle& t)
{
	std::array<Eigen::Vector4f,3> TriMax = t.toVector4(); //齐次坐标下的triangle cood

	auto& v = t.vertexs;
	auto& c = t.colors;
	auto& n = t.normals;

	//depth test optim
	int top, bottom, left, right;
	top = std::min(std::min(v[0].y(), v[1].y()), v[2].y())-1;
	top = std::max(0, top);
	bottom = std::max(std::max(v[0].y(),v[1].y()),v[2].y())+1;
	bottom = std::min(bottom, height);
	left = std::min(std::min(v[0].x(), v[1].x()),v[2].x())-1;
	left = std::max(0, left);
	right = std::max(std::max(v[0].x(), v[1].x()),v[2].x())+1;
	right = std::min(right, width);

	for (int i = top; i < bottom; ++i)
	{
		for (int j = left; j < right; ++j)
		{
			if (insideTriangle(Eigen::Vector3f(j, i, 0.0f), t))
			{
				float barycentric[3];//重心坐标
				t.getBarycentricCoords3d(Eigen::Vector3f(j, i, 0.0f), barycentric);
				int ind = i * width + j;
				float weight = (barycentric[0]/v[0].w() + barycentric[1]/v[1].w() + barycentric[2]/v[2].w());
				float depth = interpolate_f(barycentric[0], barycentric[1], barycentric[2], v[0].z(), v[1].z(), v[2].z(), weight);
				if (depth_buf[ind] > -depth)
				{
					//颜色插值
					Eigen::Vector3f color = interpolate(barycentric[0], barycentric[1], barycentric[2], c[0], c[1], c[2], weight);

					//法向量插值
					Eigen::Vector3f normal = interpolate(barycentric[0], barycentric[1], barycentric[2], n[0], n[1], n[2], weight);

					frame_buf[ind] = color;
					depth_buf[ind] = t.vertexs[0].z();
				}
			}
		}
	}
}

void Rasterizer::set_camera(Eigen::Matrix4f c)
{
	cameraMatrix = c;
}

void Rasterizer::set_model(Eigen::Matrix4f m)
{
	modelMatrix = m;
}

void Rasterizer::set_projection(Eigen::Matrix4f p )
{
	projectionMatrix = p;
}

