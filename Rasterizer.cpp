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
	using namespace::Eigen;

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	for (const auto& te : triangleList)
	{
		Triangle t(*te);
		Eigen::Vector4f v[3];
		//1.进行MCP变换
		Eigen::Matrix4f mcp =projectionMatrix * cameraMatrix * modelMatrix;
		{
			static bool test = true;
			if (test)
			{
				test = false;
				auto fq(t.vertexs[1]);
				std::cout << "origin" << std::endl;
				std::cout << fq << std::endl;
				std::cout << "after modelMatrix" << std::endl;
				std::cout << (fq = modelMatrix * fq) << std::endl;
				std::cout << "after cameraMatrix" << std::endl;
				std::cout << (fq = cameraMatrix * fq) << std::endl;
				std::cout << "projectMatrix" << std::endl;
				std::cout << projectionMatrix << std::endl;
				std::cout << "after projection" << std::endl;
				std::cout << (fq = projectionMatrix * fq) << std::endl;
			}
		}
		for (int i = 0; i < 3; ++i) v[i] = mcp*t.vertexs[i];
		std::array<Vector3f,3> objPos;//存储透视变换前，三角形在空间中的原始位置
		std::transform(&v[0], &v[2]+1, objPos.begin(), [](auto& vv) {
			return vv.template head<3>();
			}
		);
		//2.透视除法
		for (int i = 0; i < 3; ++i)
		{
			//float b = v[i].w();
			float a = 1/v[i].w();
			v[i].x() *= a;
			v[i].y() *= a;
			v[i].z() *= a;
			//v[i].x() /= v[i].w();
			//v[i].y() /= v[i].w();
			//v[i].z() /= v[i].w();
		}
		//3.法向量修正
		//4.视口变换
		for (int i = 0; i < 3; ++i)
		{
			v[i].x() = (v[i].x() + 1.0f) * width * 0.5f;
			v[i].y() = (v[i].y() + 1.0f) * height * 0.5f;
			v[i].z() = f1 * v[i].z() + f2;//扩大差异
			t.set_vertex(i, v[i]);
		}
		//5.渲染最终的三角形
		rasterize_triangle(t,objPos);
	}
}

void Rasterizer::rasterize_triangle(Triangle& t, const std::array<Eigen::Vector3f, 3>& objPos)
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
				float weight = std::abs((barycentric[0]/v[0].w() + barycentric[1]/v[1].w() + barycentric[2]/v[2].w()));
				float depth = interpolate_f(barycentric[0], barycentric[1], barycentric[2], v[0].z(), v[1].z(), v[2].z(), weight);
				depth *= weight;
				if (depth_buf[ind] > depth)
				{
					//颜色插值
					Eigen::Vector3f color = interpolate(barycentric[0], barycentric[1], barycentric[2], c[0], c[1], c[2], weight);

					//法向量插值
					Eigen::Vector3f normal = interpolate(barycentric[0], barycentric[1], barycentric[2], n[0], n[1], n[2], weight);

					//渲染位置插值
					Eigen::Vector3f viewPos = interpolate(barycentric[0], barycentric[1], barycentric[2], objPos[0], objPos[1], objPos[2], weight);

					//frame_buf[ind] = color;
					frame_buf[ind] = phongShader(color, normal, viewPos);
					depth_buf[ind] = depth;
				}
			}
		}
	}
}

Eigen::Vector3f Rasterizer::phongShader(Eigen::Vector3f color, Eigen::Vector3f normal, Eigen::Vector3f pos)
{

	using namespace::Eigen;

	normal = normal.normalized();

	Vector3f ka(0.04f, 0.05f, 0.05f);
	Vector3f ks(0.9f, 0.9f, 0.9f);
	Vector3f& kd = color;

	Vector3f Ia(1.4f, 1.4f, 1.4f);
	Vector3f lightPos(2.0f, 2.0f, 2.0f);
	Vector3f lightIntensity(7.5f, 7.5f, 7.5f);

	Vector3f result_color(0.0f, 0.0f, 0.0f);

	//result_color += ka * Ia; 这一步不能直接乘，3x1 和 3x1 的向量是不能乘的，需要做矩阵来传递
	Matrix3f ka_m = Matrix3f::Identity();
	ka_m(0, 0) = ka.x();
	ka_m(1, 1) = ka.y();
	ka_m(2, 2) = ka.z();

	//环境光
	result_color += (Ia.transpose() * ka_m).transpose(); //((3x1)T 3x3)T = 3x1

	Matrix3f kd_m = Matrix3f::Identity();
	kd_m(0, 0) = kd.x();
	kd_m(1, 1) = kd.y();
	kd_m(2, 2) = kd.z();

	//漫反射
	Vector3f lightDir = lightPos - pos;
	float cos_theat = normal.normalized().dot((lightPos - pos).normalized());
	float distanceR = (lightPos - pos).norm();
	Vector3f diffuse = 0.7 * kd_m * lightIntensity * std::max(0.0f, cos_theat) / (distanceR * distanceR);
	result_color += diffuse;

	//镜面反射
	Matrix3f ks_m = Matrix3f::Identity();
	ks_m(0, 0) = ks.x();
	ks_m(1, 1) = ks.y();
	ks_m(2, 2) = ks.z();

	int P = 32;
	Vector3f reflectDir = 2 * normal.dot(lightDir) * normal - lightDir;
	cos_theat = lightDir.normalized().dot(reflectDir.normalized());
	Vector3f reflectColor = ks_m * lightIntensity * std::pow(std::max(0.0f, cos_theat),P) / std::pow(distanceR, 2);
	result_color += reflectColor;

	result_color.x() = result_color.x() > 1.0f ? 1.0f : result_color.x();
	result_color.y() = result_color.y() > 1.0f ? 1.0f : result_color.y();
	result_color.z() = result_color.z() > 1.0f ? 1.0f : result_color.z();

	return result_color;
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

