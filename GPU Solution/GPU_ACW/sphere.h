#ifndef SPHEREH
#define SPHEREH

#include "hitable.h"

class sphere : public hitable {
public:
	__device__ sphere() {}
	__device__ sphere(vec3 cen, vec3 vel) : center(cen), vel(vel) {};
	__device__ sphere(vec3 cen, float r) : center(cen), radius(r), vel(vec3(0,0,0)) {};
	__device__ virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;

	__device__ virtual void move(int index);
	__device__ virtual void reVel(int index);

	__device__ virtual void applyGravity();
	__device__ virtual void letChaos(int index, vec3 newVel);

	vec3 center;
	vec3 vel;
	float force = 23.45;
	float radius = 1;
};

__device__ void sphere::move(int index) {
	center = center + vel;
	//printf("(%f, %f, %f)\n", vel.x(), vel.y(), vel.z());
}

__device__ void sphere::reVel(int index) {
    float xloc = center.x() + vel.x();
    float yloc = center.y() + vel.y();
    float zloc = center.z() + vel.z();
	if (xloc < -49 || xloc > 49)
		vel = vec3(vel.x() * -1, vel.y(), vel.z());
    if (yloc < -49 || yloc > 49)
        vel = vec3(vel.x() ,vel.y() * -1, vel.z());
    if (zloc < -100 || zloc > 1)
        vel = vec3(vel.x(), vel.y(), vel.z() * -1);
}

__device__ void sphere::applyGravity() {
	if (center.y() + vel.y() >= -48)
		vel = vec3(vel.x(), vel.y() - 0.5, vel.z());
	else if (center.y() + vel.y() < -48 && vel.y() < -0.5)
		vel = vec3(vel.x(), abs(vel.y()) - 0.5, vel.z());
	else {
		vel = vec3(0, 0, 0);
	}
}

__device__ void sphere::letChaos(int index, vec3 newVel) {
	if(vel.x() == 0 && vel.y() == 0 && vel.z() == 0)
		vel = newVel;
}

////////////////////////////////





//////////////////////////////////////


__device__ bool sphere::hit(const ray& r, float t_min, 
			float t_max, hit_record& rec) const 
{
	vec3 oc = r.origin() - center;
	float a = dot(r.direction(), r.direction());
	//printf("a = %.6f \n", a);
	float b = dot(oc, r.direction());
	//printf("b = %.6f \n", b);
	float c = dot(oc, oc) - radius * radius;
	//printf("c = %.6f \n", c);
	float discriminant = b * b - a * c;
	//printf("dis = %.6f \n", discriminant);
	if (discriminant > 0) {
		float temp = (-b - sqrt(discriminant)) / a;
		if (temp < t_max && temp > t_min) {
			rec.t = temp;
			rec.p = r.point_at_parameter(rec.t);
			rec.normal = (rec.p - center) / radius;
			return true;
		}
		temp = (-b + sqrt(discriminant)) / a;
		if (temp < t_max && temp > t_min) {
			rec.t = temp;
			rec.p = r.point_at_parameter(rec.t);
			rec.normal = (rec.p - center) / radius;
			return true;
		}
	}
	return false;
}


#endif