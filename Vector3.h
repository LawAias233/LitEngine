#pragma once
#include <cmath>
class Vector3
{
public:
	float x, y, z;

	//constructor
	Vector3():x(0),y(0),z(0) {}
	Vector3(float _k) :x(_k), y(_k), z(_k) {}
	Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
	Vector3(const Vector3& _v) : x(_v.x), y(_v.y), z(_v.z) {}

	Vector3& operator=(const Vector3& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}

	bool operator==(const Vector3& v)
	{
		return x == v.x && y == v.y && z == v.z;
	}

	bool operator!=(const Vector3& v)
	{
		return x != v.x || y != v.y || z != v.z;
	}

	void zero() { x = y = z = 0.0f; }

	Vector3 operator-() const
	{
		return Vector3(-x, -y, -z);
	}

	Vector3& operator+=(const Vector3& v)
	{
		x += v.x; y += v.y; z += v.z;
		return *this;
	}

	Vector3& operator-=(const Vector3& v)
	{
		x -= v.x; y -= v.y; z -= v.z;
		return *this;
	}

	Vector3& operator*=(float k)
	{
		x *= k; y *= k; z *= k;
		return *this;
	}

	Vector3& operator/=(float k)
	{
		float rk = 1 / k;
		return *this *= rk;
	}

	Vector3 normalize()
	{
		float magSq = x * x + y * y + z * z;
		if (magSq > 0.0f)
		{
			magSq = 1/sqrt(magSq);
			float _x = x * magSq;
			float _y = y * magSq;
			float _z = z * magSq;
			return Vector3(_x, _y, _z);
		}
		return *this;
	}

	float magnitude()
	{
		return sqrt(x * x + y * y + z * z);
	}
};

//extend ostream
std::ostream& operator<<(std::ostream& out, const Vector3& v)
{
	out << v.x << ' ' << v.y << ' ' << v.z;
	return out;
}

//inline operator
inline Vector3 operator+(const Vector3& u, const Vector3& v) 
{
	return Vector3(u.x + v.x, u.y + v.y, u.z + v.z);
}

inline Vector3 operator-(const Vector3& u, const Vector3& v) 
{
	return Vector3(u.x - v.x, u.y - v.y, u.z - v.z);
}

inline Vector3 operator*(float k, const Vector3& u) 
{
	return Vector3(u.x * k, u.y * k, u.z * k);
}

inline Vector3 operator*(const Vector3& u, float k) 
{
	return Vector3(u.x * k, u.y * k, u.z * k);
}

inline Vector3 operator/(float k, const Vector3& u)
{
	return Vector3(u.x / k, u.y / k, u.z / k);
}

inline Vector3 operator/(const Vector3& u, float k)
{
	return Vector3(u.x / k, u.y / k, u.z / k);
}

inline float operator*(const Vector3& u, const Vector3& v)
{
	return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline float dotProduct(const Vector3& u, const Vector3& v)
{
	return u * v;
}

inline Vector3 crossProduct(const Vector3& u, const Vector3& v)
{
	/*| i   j   k   |
	* | u_x u_y u_z |
	* | v_x v_y v_z |
	
	float i = u.y * v.z - u.z * v.y;
	float j = -(u.x * v.z - u.z * v.x);
	float k = u.x * v.y - u.y * v.x;
	return Vector3(i, j, k);
	*/
	return Vector3(
		u.y * v.z - u.z * v.y,
		u.z * v.x - u.x * v.z,
		u.x * v.y - u.y * v.x
	);
}

inline float distance(const Vector3& u, const Vector3& v)
{
	float dx = u.x - v.x;
	float dy = u.y - v.y;
	float dz = u.z - v.z;
	return sqrt(dx * dx + dy * dy + dz * dz);
}