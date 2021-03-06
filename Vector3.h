#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>

class Vector3 // simplesmente um vetor com 3 floats x, y e z.
{	
public:
	float f[3];

	Vector3(float x, float y, float z)
	{
		f[0] =x;
		f[1] =y;
		f[2] =z;
	}

	Vector3() {}

	float length()
	{
		return sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
	}

	Vector3 normalized()
	{
		float l = length();
		return Vector3(f[0]/l,f[1]/l,f[2]/l);
	}

	void operator+= (const Vector3 &v)
	{
		f[0]+=v.f[0];
		f[1]+=v.f[1];
		f[2]+=v.f[2];
	}

	Vector3 operator/ (const float &a)
	{
		return Vector3(f[0]/a,f[1]/a,f[2]/a);
	}

	Vector3 operator- (const Vector3 &v)
	{
		return Vector3(f[0]-v.f[0],f[1]-v.f[1],f[2]-v.f[2]);
	}

	Vector3 operator+ (const Vector3 &v)
	{
		return Vector3(f[0]+v.f[0],f[1]+v.f[1],f[2]+v.f[2]);
	}

	Vector3 operator* (const float &a)
	{
		return Vector3(f[0]*a,f[1]*a,f[2]*a);
	}

	Vector3 operator-()
	{
		return Vector3(-f[0],-f[1],-f[2]);
	}

	Vector3 cross(const Vector3 &v)
	{
		return Vector3(f[1]*v.f[2] - f[2]*v.f[1], f[2]*v.f[0] - f[0]*v.f[2], f[0]*v.f[1] - f[1]*v.f[0]);
	}

	float dot(const Vector3 &v)
	{
		return f[0]*v.f[0] + f[1]*v.f[1] + f[2]*v.f[2];
	}
};

#endif