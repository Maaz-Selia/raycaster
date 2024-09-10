#pragma once

#ifndef HITABLEH
#define HITABLEH

#include "ray.h"

struct hit_record
{
	float t;
	vec3 p;
	vec3 normal;
};

class hitable {
public:
	__device__ virtual bool hit(const ray& r, float t_min, float t_max, hit_record& rec) const = 0;

	__device__ virtual void move(int index) = 0;
	__device__ virtual void reVel(int index) = 0;

	__device__ virtual void applyGravity() = 0;
	__device__ virtual void letChaos(int index, vec3 newVel) = 0;

};

#endif