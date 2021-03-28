//#define TEST

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>

#include "Rasterizer.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION 
#include "stb_image_write.h"

//#include "stb_image.h"
#include "OBJ_Loader.h"

using namespace::std;

Eigen::Matrix4f get_modelMatrix(float angle); //暂时只支持旋转
Eigen::Matrix4f get_cameraMatrix(Eigen::Vector3f viewpos, Eigen::Vector3f look_dir, Eigen::Vector3f look_up);
Eigen::Matrix4f get_projectionMatrix(float fov, float aspect_ratio, float zNear, float zFar);

const float PI = 3.1415926;
#ifndef TEST

int main()
{
	
	const int width(800), height(600), cmp(3);
	string filepath("out.png");
	string texturepath("models/spot/hmap.jpg");
	vector<Triangle*>triangleList;

	std::shared_ptr<Texture>texture(new Texture(texturepath.c_str()));
	objl::Loader loader;
	bool loadout = loader.LoadFile("models/spot/spot_triangulated_good.obj");
	for (auto mesh : loader.LoadedMeshes)
	{
		for (int i = 0; i < mesh.Vertices.size(); i += 3)
		{
			Triangle* t = new Triangle();
			for (int j = 0; j < 3; j++)
			{
				t->set_vertex(j, Eigen::Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0f));
				t->set_normal(j, Eigen::Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
				t->set_texCoord(j, Eigen::Vector2f(mesh.Vertices[i + j].TextureCoordinate.X, mesh.Vertices[i + j].TextureCoordinate.Y));
				t->set_color(j, 0x66, 0xCC, 0xFF);
			}
			triangleList.push_back(t);
		}
	}

	
	Rasterizer r(width, height);
	unique_ptr<unsigned char[]>data(new unsigned char[width*height*cmp]);
	
	//cout << get_modelMatrix(45) << endl;
	//cout << get_cameraMatrix(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, -1.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f)) << endl;
	//cout << get_projectionMatrix(45, 1.333, 0.1, 50) << endl;
	//return 0;
	r.set_texture(texture);
	r.set_model(get_modelMatrix(-90));
	r.set_camera(get_cameraMatrix(Eigen::Vector3f(0.0f,0.0f,-2.5f), Eigen::Vector3f(0.0f,0.0f,1.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f)));
	r.set_projection(get_projectionMatrix(45,1.333,-0.1,-50));
	//r.rasterize(triangleList[0]);
	r.rasterize(triangleList);

	//r.rasterize_triangle(t);
	//r.rasterize_triangle(t2);
	int dataIndex(0);
	for (auto& it : r.get_frame_buf())
	{
		data.get()[dataIndex+0] = it.x()*255.99f;
		data.get()[dataIndex+1] = it.y()*255.99f;
		data.get()[dataIndex+2] = it.z()*255.99f;
		dataIndex += cmp;
	}

	stbi_write_png(filepath.c_str(), width, height, cmp, data.get(), width * cmp);

	
	return 0;
}

#endif // !TEST

Eigen::Matrix4f get_modelMatrix(float angle) //暂时只支持绕y轴旋转
{
	Eigen::Matrix4f translate, rotation, scale;

	translate << 
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	angle = angle * PI / 180;
	rotation <<
		cos(angle), 0.0f, -sin(angle), 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		-sin(angle), 0.0f, cos(angle), 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	scale <<
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f;

	return translate* rotation * scale;
}

Eigen::Matrix4f get_cameraMatrix(Eigen::Vector3f viewpos, Eigen::Vector3f look_dir, Eigen::Vector3f look_up)
{
	Eigen::Vector3f w = -look_dir.normalized();
	Eigen::Vector3f u = look_up.cross(w);
	u.normalize();
	Eigen::Vector3f v = w.cross(u);
	Eigen::Matrix4f view, translate;
	view << 
		u.x(), u.y(), u.z(), 0.0f,
		v.x(), v.y(), v.z(), 0.0f,
		w.x(), w.y(), w.z(), 0.0f,
		0.0f , 0.0f , 0.0f , 1.0f;
	translate <<
		1.0f, 0.0f, 0.0f, -viewpos.x(),
		0.0f, 1.0f, 0.0f, -viewpos.y(),
		0.0f, 0.0f, 1.0f, -viewpos.z(),
		0.0f, 0.0f, 0.0f, 1.0f;
	return view * translate;
}

Eigen::Matrix4f get_projectionMatrix(float fov, float aspect_ratio, float zNear, float zFar)
{
	Eigen::Matrix4f perspective, orthographic, projectionMatrix;
	float l, b, n(zNear);
	float r, t, f(zFar);
	float fov_angle = fov * PI / 180;
	t = n * tan(fov_angle * 0.5f);
	b = -t;
	r = t * aspect_ratio;
	l = -r;

	projectionMatrix << 
		2*n/(r-l), 0.0f, 0.0f, 0.0f,
		0.0f, 2*n/(t-b), 0.0f, 0.0f,
		0.0f, 0.0f, (n+f)/(n-f), -2*n*f/(n-f),
		0.0f, 0.0f, 1.0f, 0.0f;
	//return projectionMatrix;
	orthographic <<
		2.0f / (r - l), 0.0f, 0.0f, -(r + l) / (r - l),
		0.0f, 2.0f / (t - b), 0.0f, -(t + b) / (t - b),
		0.0f, 0.0f, 2.0f / (n - f), -(n + f) / (n - f),
		0.0f, 0.0f, 0.0f, 1.0f;
	perspective <<
		n, 0.0f, 0.0f, 0.0f,
		0.0f, n, 0.0f, 0.0f,
		0.0f, 0.0f, n + f, -n * f,
		0.0f, 0.0f, 1.0f, 0.0f;
	return orthographic * perspective;
	//return orthographic ;
}
#ifdef TEST

int main()
{
	Texture t("models/spot/hmap.jpg");
	int dataIndex(0);
	for (int i = 0; i < t.h; i++)
	{
		for (int j = 0; j < t.w; j++)
		{
			t.data[dataIndex + 0] += 55.0f;
			t.data[dataIndex + 1] += 55.0f;
			t.data[dataIndex + 2] += 55.0f;
			dataIndex += 3;
		}
	}

	stbi_write_png("out2.png", t.w, t.h, t.n, t.data, t.w * t.n);

	return 0;
}
#endif // TEST
