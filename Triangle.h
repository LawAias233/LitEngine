#pragma once
#include <vector>
#include <array>
#include <Eigen/Dense>
class Triangle
{
	/* һ������������Ҫʲô��
	*  3*Vector3f--�������������--��Ҫ����
	*  3*Vector3f--�����������ɫ--��Ҫ����
	*  3*Vector2f--�����ε���������--�ڲ�����--��Ϊ�������Ǹ�ƽ�棬���Ի���Ϊ2D���
	*  3*Vector3f--����������Եķ�����
	*/
public:
	Triangle();
	void set_vertex(int ind, Eigen::Vector4f);
	void set_color(int ind, float, float, float);
	void set_normal(int ind, Eigen::Vector3f);
	void set_texCoord(int ind, Eigen::Vector2f);

	Eigen::Vector3f get_color() { return colors[0]; }//��ʱ����������Ϊ��ɫ
	std::array<Eigen::Vector4f, 3> toVector4() const; //�����������ϵ������������
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

