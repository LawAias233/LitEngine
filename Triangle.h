#pragma once
#include <vector>
#include <array>
#include <Eigen/Dense>
class Triangle
{
	/* 一个三角形类需要什么？
	*  3*Vector3f--三个顶点的坐标--需要输入
	*  3*Vector3f--三个顶点的颜色--需要输入
	*  3*Vector2f--三角形的重心坐标--内部计算--因为三角形是个平面，所以化简为2D情况
	*  3*Vector3f--三个顶点各自的法向量
	*/
public:
	Triangle();
	void set_vertex(int ind, Eigen::Vector4f);
	void set_color(int ind, float, float, float);
	void set_normal(int ind, Eigen::Vector3f);
	void set_texCoord(int ind, Eigen::Vector2f);

	Eigen::Vector3f get_color() { return colors[0]; }//暂时假设三角形为纯色
	std::array<Eigen::Vector4f, 3> toVector4() const; //返回齐次坐标系的三角形坐标
	bool getBarycentricCoords3d(Eigen::Vector3f p, float[3]);
public:
	Eigen::Vector4f vertexs[3];
	Eigen::Vector3f colors[3];
	Eigen::Vector2f tex_coords[3];
	Eigen::Vector3f normals[3];
};

inline bool insideTriangle(Eigen::Vector3f p, Triangle& t)
{
	float f[3];
	t.getBarycentricCoords3d(p,f);
	int a = 1;
	if (f[0] < 0 || f[1] < 0 || f[2] < 0) 
		return false;
	return true;
}

