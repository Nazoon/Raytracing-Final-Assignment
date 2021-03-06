#include "raycolor.h"
#include "first_hit.h"
#include "blinn_phong_shading.h"
#include "reflect.h"
#include <math.h>
#include <Eigen/Core>
#include <stdio.h>
#include <iostream>

/*
Find the transmittance and reflectance values for a refractive material and an incident ray
https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
*/
void find_transmittance_and_reflectance(
	Eigen::Vector3d i,
	Eigen::Vector3d n,
	double eta1,
	double eta2,
	double& T,
	double& R)
{
	Eigen::Vector3d ray_dot_n_times_n = i.dot(n) * n;
	double ratio = eta1 / eta2;
	double cos_i = ray_dot_n_times_n.norm();
	double sin_i = (i - ray_dot_n_times_n).norm();
	double sin2_t = ratio * ratio * (1 - cos_i * cos_i);
	double cos_t = std::sqrt(1 - sin2_t);
	double R_0 = (eta1 - eta2) / (eta1 + eta2);
	R_0 *= R_0;
	if (eta1 <= eta2) {
		double x = 1.0 - cos_i;
		R = R_0 + (1.0 - R_0) * x * x * x * x * x;
	}
	else {
		bool total_internal_reflection = sin_i > (eta2 / eta1);
		if (total_internal_reflection) {
			R = 1.0;
			T = 0.0;
			return;
		}
		else {
			double x = 1.0 - cos_t;
			R = R_0 + (1.0 - R_0) * x * x * x * x * x;
		}
	}
	T = 1.0 - R;

	if (T > 1.0) T = 1.0;
	if (R < 0.0) R = 0.0;

	T = 1.0;
	R = 0.0;
}

/*
Compute the caustics at a given point
*/
Eigen::Vector3d caustics_at_point(
	Eigen::Vector3d center,
	const std::shared_ptr<KDTree> light_map_tree
) {
	// Also compute light from caustics
	std::vector<LightPoint> light_points;
	std::vector<double> sdists;
	light_map_tree->get_points_in_range(center, light_map_range, light_points, sdists);
	double max_sdist = light_map_range * light_map_range;
	Eigen::Vector3d caustic_rgb(0, 0, 0);
	for (int i = 0; i < sdists.size(); i++) {
		double dist_factor = ((max_sdist - sdists[i]) / (max_sdist));
		if (dist_factor < 0) dist_factor = 0;
		caustic_rgb += light_points[i].rgb * dist_factor;
	}
	return caustic_rgb;
}

bool raycolor(
	const Ray& ray,
	const double min_t,
	const std::vector< std::shared_ptr<Object> >& objects,
	const std::vector< std::shared_ptr<Light> >& lights,
	const int num_recursive_calls,
	const std::shared_ptr<KDTree> light_map_tree,
	Eigen::Vector3d& rgb)
{
	int hit_id;
	double t;
	Eigen::Vector3d n;
	if (first_hit(ray, min_t, objects, hit_id, t, n)) {

		// Basic shading
		rgb += blinn_phong_shading(ray, hit_id, t, n, objects, lights);
		
		// Also compute light from caustics
		rgb += caustics_at_point(ray.origin + (t * ray.direction), light_map_tree);

		// This is the raytracing part
		if (num_recursive_calls <= max_num_recursive_calls) {

			if (objects[hit_id]->material->refractive_index != -1) {

				// Refractive material!

				// Checking for exiting a translucent material
				double eta1 = ray.cur_medium_refractive_index;
				double eta2 = objects[hit_id]->material->refractive_index;
				if (eta1 == eta2) {
					// We assume the ray to be exiting the material into air.
					// This means we are not allowed to have overlapping translucent materials.
					eta2 = 1.0;
				}

				// Setting reflectance and transmittance variables
				double T, R;
				find_transmittance_and_reflectance(ray.direction, n, eta1, eta2, T, R);

				// Combining relfected ray and refracted ray
				// Relfected light
				if (R > 0.0) {
					Eigen::Vector3d reflect_rgb(0, 0, 0);
					Ray reflect_ray;
					reflect_ray.origin = ray.origin + (t * ray.direction);
					reflect_ray.direction = reflect(ray.direction, n);
					reflect_ray.cur_medium_refractive_index = eta1;
					if (raycolor(reflect_ray, fudge, objects, lights, num_recursive_calls + 1, light_map_tree, reflect_rgb)) {
						for (int i = 0; i < 3; i++) {
							rgb[i] += reflect_rgb[i] * R * objects[hit_id]->material->km[i] * objects[hit_id]->material->opacity[i];
						}
					}
				}
				// Refracted light
				if (T > 0.0) {
					Eigen::Vector3d refract_rgb(0, 0, 0);
					Ray refract_ray;
					refract_ray.origin = ray.origin + (t * ray.direction);
					refract_ray.direction = refract(ray.direction, n, eta1, eta2);
					refract_ray.cur_medium_refractive_index = eta2;
					if (raycolor(refract_ray, fudge, objects, lights, num_recursive_calls + 1, light_map_tree, refract_rgb)) {
						for (int i = 0; i < 3; i++) {
							rgb[i] += refract_rgb[i] * T * (1.0 - objects[hit_id]->material->opacity[i]);
						}
					}
				}
			}
			else {
				// Opaque material, compute reflected light
				Eigen::Vector3d reflect_rgb(0, 0, 0);
				Ray reflect_ray;
				reflect_ray.origin = ray.origin + (t * ray.direction);
				reflect_ray.direction = reflect(ray.direction, n);
				reflect_ray.cur_medium_refractive_index = ray.cur_medium_refractive_index;
				if (raycolor(reflect_ray, fudge, objects, lights, num_recursive_calls + 1, light_map_tree, reflect_rgb)) {
					for (int i = 0; i < 3; i++) {
						rgb[i] += reflect_rgb[i] * objects[hit_id]->material->km[i];
					}
				}
			}
		}

		return true;
	}
	return false;
}

/*
Sets up a light map for caustics
http://www.follick.ca/rt/

Inputs: Mostly the same as raycolour, with the addition of ray_rgb so we may know the colour of the light ray.
Outputs: light_points, the "light map" which is to be passed into a KDTree for range checking after.
*/
void cast_light(
	const Ray& ray,
	const Eigen::Vector3d ray_rgb,
	const double min_t,
	const std::vector< std::shared_ptr<Object> >& objects,
	const int num_recursive_calls,
	std::vector<LightPoint>& light_points)
{
	if (num_recursive_calls <= max_num_recursive_calls) {
		int hit_id;
		double t;
		Eigen::Vector3d n;
		if (first_hit(ray, min_t, objects, hit_id, t, n)) {

			if (objects[hit_id]->material->refractive_index != 1) {
				// Refractive object, split light ray into two new ones based on transmittance and reflectance

				// Checking for exiting a translucent material
				double eta1 = ray.cur_medium_refractive_index;
				double eta2;
				if (eta1 == 1.0 || eta1 == -1.0) {
					eta2 = objects[hit_id]->material->refractive_index;
					//printf("entering...\n");
				}
				else {
					// We assume the ray to be exiting the material into air.
					// This means we are not allowed to have overlapping translucent materials.
					eta2 = 1.0;
					//printf("n1 = %f n2 = %f\n", eta1, eta2);
				}

				// Setting reflectance and transmittance variables
				double T, R;
				find_transmittance_and_reflectance(ray.direction, n, eta1, eta2, T, R);

				// Casting relfected ray and refracted ray
				// Relfected light
				if (R > 0.0) {
					Ray reflect_ray;
					reflect_ray.origin = ray.origin + ((t + fudge) * ray.direction);
					reflect_ray.direction = reflect(ray.direction, n);
					reflect_ray.cur_medium_refractive_index = eta1;
					cast_light(reflect_ray, ray_rgb * R, min_t, objects, num_recursive_calls + 1, light_points);
				}
				// Refracted light
				if (T > 0.0) {
					//printf("refracting...\n");
					Ray refract_ray;
					refract_ray.origin = ray.origin + ((t + fudge) * ray.direction);
					refract_ray.direction = refract(ray.direction, n, eta1, eta2);
					refract_ray.cur_medium_refractive_index = eta2;
					cast_light(refract_ray, ray_rgb * T, min_t, objects, num_recursive_calls + 1, light_points);
				}
			}
			else {

				// "Deposit" the rest of the light ray into the light map if this isn't the ray's first redirection.
				if (num_recursive_calls > 0) {
					LightPoint caustic;
					caustic.pos = ray.origin + (t * ray.direction);
					for (int i = 0; i < 3; i++) {
						caustic.rgb[i] = ray_rgb[i] * objects[hit_id]->material->ks[i];
					}
					light_points.emplace_back(caustic);
					//printf("caustic on object %d at pos:\n", hit_id);
					//std::cout << caustic.pos << std::endl;
					//printf("rgb =\n");
					//std::cout << caustic.rgb << std::endl;
					//printf("depth = %d\n", num_recursive_calls);
				}

				// Not refractive, reflect some light into a new light ray
				Eigen::Vector3d new_ray_rgb;
				for (int i = 0; i < 3; i++) {
					new_ray_rgb[i] = ray_rgb[i] * objects[hit_id]->material->km[i];
				}
				//Ray reflect_ray;
				//reflect_ray.origin = ray.origin + (t * ray.direction);
				//reflect_ray.direction = reflect(ray.direction, n);
				//reflect_ray.cur_medium_refractive_index = ray.cur_medium_refractive_index;
				//cast_light(reflect_ray, new_ray_rgb, min_t, objects, num_recursive_calls + 1, light_points);
			}
		}
	}
}

/*
Given the min and max corners of an AABB, insert another point into it.
*/
void insert_point_into_box(
	Eigen::Vector3d& min,
	Eigen::Vector3d& max,
	Eigen::Vector3d pos)
{
	//std::cout << pos << std::endl;
	for (int d = 0; d < 3; d++) {
		min[d] = pos[d] < min[d] ? pos[d] : min[d];
		max[d] = pos[d] > max[d] ? pos[d] : max[d];
	}
}