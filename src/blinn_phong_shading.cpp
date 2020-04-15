#include "blinn_phong_shading.h"
// Hint:
#include "first_hit.h"
#include <iostream>

// Magic number used to prevent self-shadowing
const double fudge = 0.000001;

// Helper function for element-wise vector multiplication
Eigen::Vector3d v_multiply(Eigen::Vector3d a, Eigen::Vector3d b) {
	Eigen::Vector3d c;
	for (int i = 0; i < 3; i++) {
		c[i] = a[i] * b[i];
	}
	return c;
}

Eigen::Vector3d blinn_phong_shading(
	const Ray& ray,
	const int& hit_id,
	const double& t,
	const Eigen::Vector3d& n,
	const std::vector< std::shared_ptr<Object> >& objects,
	const std::vector<std::shared_ptr<Light> >& lights)
{
	int shadow_hit_id;
	double max_t, obj_t, p;
	Eigen::Vector3d rgb, hit_pos, shadow_n, h, L, I, kd, ks;
	Ray l; // Ray from hit pos to light sources

	// Initial ambient colour
	hit_pos = ray.origin + (t * ray.direction);
	rgb = objects[hit_id]->material->ka * 0.1;
	kd = objects[hit_id]->material->kd;
	ks = objects[hit_id]->material->ks;
	p = objects[hit_id]->material->phong_exponent;

	for (int i = 0; i < lights.size(); i++) {

		// Getting direction from hit_pos to current light source
		l.origin = hit_pos;
		lights[i]->direction(hit_pos, l.direction, max_t);

		// True iff l does not intersect with any object on its way to the current light source
		if (!first_hit(l, fudge, objects, shadow_hit_id, obj_t, shadow_n) || max_t < obj_t) {

			// Intensity of this light
			I = lights[i]->I;

			// Diffuse lighting
			rgb += v_multiply(v_multiply(kd, I) * std::max(0.0, n.transpose().dot(l.direction)), objects[hit_id]->material->opacity);

			// Specular lighting
			h = (l.direction - ray.direction.normalized());
			h.normalize();
			L = ks * pow(std::max(0.0, n.dot(h)), p);
			rgb += v_multiply(v_multiply(L, I), objects[hit_id]->material->opacity);
		}

	}
	return rgb;
}
