#ifndef HITABLELISTH
#define HITABLELISTH

#include "hitable.h"

class hitable_list : public hitable {
public:
	__device__ hitable_list() {}
	__device__ hitable_list(hitable **l, int n) { list = l; list_size = n; }
	__device__ virtual bool hit(const ray& r, float tmin, float tmax, hit_record& rec) const;

	__device__ virtual void move(int index);
	__device__ virtual void reVel(int index);

	__device__ virtual void applyGravity();
	__device__ virtual void letChaos(int index, vec3 newVel);

	hitable **list;
	int list_size;
};


__device__ void hitable_list::move(int index) {
	list[index]->move(index);
}

__device__ void hitable_list::reVel(int index) {
	list[index]->reVel(index);
}

__device__ void hitable_list::applyGravity() {
	for (int i = 0; i < list_size; i++) {
		list[i]->applyGravity();
	}
}
__device__ void hitable_list::letChaos(int index, vec3 newVel) {
	list[index]->letChaos(index, newVel);
}


__device__ bool hitable_list::hit(const ray& r, float t_min, float t_max, hit_record& rec) const {
	hit_record temp_rec;
	bool hit_anything = false;
	float closest_so_far = t_max;
	for (int i = 0; i < list_size; i++) {
		if (list[i]->hit(r, t_min, closest_so_far, temp_rec)) {
			hit_anything = true;
			closest_so_far = temp_rec.t;
			rec = temp_rec;
		}
	}
	return hit_anything;
}

#endif