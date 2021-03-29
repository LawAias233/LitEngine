#include "Rasterizer.h"
#include <algorithm>
#include <iostream>
#include <stdio.h>

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

static Eigen::Vector3f mcp_transform(const Eigen::Matrix4f& mcp, const Eigen::Vector3f& v)
{
	return (mcp * Eigen::Vector4f(v.x(), v.y(), v.z(), 1.0f)).head<3>();
}



void Rasterizer::rasterize(std::vector<Triangle*>triangleList)
{
	using namespace::Eigen;

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mcp = projectionMatrix * cameraMatrix * modelMatrix;

	this->cameraPos = mcp_transform(mcp, this->cameraPos);
	for (auto& light : lights) light.lightPos = mcp_transform(mcp, light.lightPos);
	
	for (const auto& te : triangleList)
	{
		Triangle t(*te);

		Eigen::Vector4f v[3];
		//1.进行MCP变换
		{
			static bool test = false;
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
			float a = 1/v[i].w();
			v[i].x() *= a;
			v[i].y() *= a;
			v[i].z() *= a;
		}
		//3.法向量修正
		Eigen::Matrix4f inv_m = (cameraMatrix* modelMatrix).inverse().transpose();
		Eigen::Vector4f n[3] = {
			inv_m * Eigen::Vector4f(te->normals[0].x(), te->normals[0].y(),te->normals[0].z(), 0.0f),
			inv_m * Eigen::Vector4f(te->normals[1].x(), te->normals[1].y(),te->normals[1].z(), 0.0f),
			inv_m * Eigen::Vector4f(te->normals[2].x(), te->normals[2].y(),te->normals[2].z(), 0.0f),
		};

		for(int i = 0; i < 3; ++i)
			t.set_normal(i, n[i].head<3>());

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
	auto& c = t.colors; //注：颜色来自Triangle类型，它在获取颜色时会归一到[0,1]之间
	auto& tex_c = t.tex_coords;
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
				//int ind = (height - 1 - i) * width + j;
				float weight = std::abs((barycentric[0]/v[0].w() + barycentric[1]/v[1].w() + barycentric[2]/v[2].w()));
				float depth = interpolate_f(barycentric[0], barycentric[1], barycentric[2], v[0].z(), v[1].z(), v[2].z(), weight);
				depth *= weight;
				if (depth_buf[ind] > depth)
				{
					//颜色插值
					Eigen::Vector3f color = interpolate(barycentric[0], barycentric[1], barycentric[2], c[0], c[1], c[2], weight);

					//法向量插值
					Eigen::Vector3f normal = interpolate(barycentric[0], barycentric[1], barycentric[2], n[0], n[1], n[2], weight).normalized();

					//渲染位置插值
					Eigen::Vector3f viewPos = interpolate(barycentric[0], barycentric[1], barycentric[2], objPos[0], objPos[1], objPos[2], weight);

					//纹理坐标插值
					Eigen::Vector2f texCoods = interpolate(barycentric[0], barycentric[1], barycentric[2], tex_c[0], tex_c[1], tex_c[2], weight);
					texCoods *= weight;

					//fragment_shader_payload payload(viewPos, color, normal, tex_c[0], this->texture);
					fragment_shader_payload payload(viewPos, color, normal, texCoods, this->texture);

					//frame_buf[ind] = color;
					//frame_buf[ind] = phongShader(payload, this->lights);
					frame_buf[ind] = textureShader(payload, this->lights);
					depth_buf[ind] = depth;
				}
			}
		}
	}
}

Eigen::Vector3f Rasterizer::phongShader(const fragment_shader_payload& payload, const std::vector<Light>& lights)
{
	Eigen::Vector3f ka(0.04f, 0.05f, 0.05f);
	Eigen::Vector3f ks(0.7f, 0.7f, 0.7f);
	Eigen::Vector3f kd = payload.color;

	Eigen::Vector3f Ia(1.0f, 1.0f, 1.0f);

	Eigen::Vector3f result_color(0.0f, 0.0f, 0.0f);
	Eigen::Matrix3f ka_m = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f kd_m = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f ks_m = Eigen::Matrix3f::Identity();
	ka_m(0, 0) = ka.x();
	ka_m(1, 1) = ka.y();
	ka_m(2, 2) = ka.z();
	kd_m(0, 0) = kd.x();
	kd_m(1, 1) = kd.y();
	kd_m(2, 2) = kd.z();
	ks_m(0, 0) = ks.x();
	ks_m(1, 1) = ks.y();
	ks_m(2, 2) = ks.z();

	//环境光
	result_color += (Ia.transpose() * ka_m).transpose(); 

	for (const auto& light : lights)
	{
		//漫反射
		Eigen::Vector3f lightDir = (light.lightPos - payload.view_pos).normalized();
		float cos_theat = payload.normal.dot(lightDir);
		float distanceR = (light.lightPos - payload.view_pos).norm();
		Eigen::Vector3f diffuse = kd_m * light.lightIntensity * std::max(0.0f, cos_theat) / std::pow(distanceR, 2);
		result_color += diffuse;
		//std::cout << diffuse << std::endl;

		//镜面反射
		int P = 16;
		Eigen::Vector3f reflectDir = payload.normal.dot(lightDir) * payload.normal - lightDir;
		cos_theat = (this->cameraPos - payload.view_pos).normalized().dot(reflectDir.normalized());
		Eigen::Vector3f reflectColor = ks_m * light.lightIntensity * std::pow(std::max(0.0f, cos_theat), P) / std::pow(distanceR, 2);
		result_color += reflectColor;
	}

	result_color.x() = result_color.x() > 1.0f ? 1.0f : result_color.x();
	result_color.y() = result_color.y() > 1.0f ? 1.0f : result_color.y();
	result_color.z() = result_color.z() > 1.0f ? 1.0f : result_color.z();

	return result_color;
}

Eigen::Vector3f getColorFromTexture(std::shared_ptr<Texture> texture_p, Eigen::Vector2f uv)
{
	float r, g, b;
	int x = texture_p->w * uv.x();
	int y = texture_p->h * (1- uv.y());
	int index = (y * texture_p->w + x)*3;
	r = texture_p->data[index+0];
	g = texture_p->data[index+1];
	b = texture_p->data[index+2];
	//printf("%f %f %f\n", r, g, b);
	return Eigen::Vector3f(r,g,b);
}

Eigen::Vector3f Rasterizer::textureShader(const fragment_shader_payload& _payload, const std::vector<Light>& lights)
{
	fragment_shader_payload payload = _payload;
	payload.color = getColorFromTexture(payload.texture, payload.texture_coords)/255.99f;
	return phongShader(payload, this->lights);
}

void Rasterizer::set_camera(const Eigen::Matrix4f& c, const Eigen::Vector3f& p)
{
	cameraMatrix = c;
	cameraPos = p;
}

void Rasterizer::set_model(const Eigen::Matrix4f& m)
{
	modelMatrix = m;
}

void Rasterizer::set_projection(const Eigen::Matrix4f& p )
{
	projectionMatrix = p;
}

void Rasterizer::set_texture(std::shared_ptr<Texture> t)
{
	texture = t;
}