#include "viewing_ray.h"
#include <Eigen/Core>

double u_coeff(int j, double width, int n_x) {
	return width * (((double)j + 0.5) / (double)n_x) - (width / 2);
}

double v_coeff(int i, double height, int n_y) {
	return height * (((double)i + 0.5) / (double)n_y) - (height / 2);
}

void viewing_ray(
	const Camera& camera,
	const int i,
	const int j,
	const int width,
	const int height,
	Ray& ray)
{
	Eigen::Vector3d u = u_coeff(j, camera.width, width) * camera.u;
	Eigen::Vector3d v = -v_coeff(i, camera.height, height) * camera.v;
	Eigen::Vector3d neg_d = -camera.w * camera.d;

	ray.origin = camera.e;
	ray.direction = (u + v + neg_d);
	ray.cur_medium_refractive_index = 1.0;

	// Assuming the ray starts in a vacuum
	ray.cur_medium_refractive_index = 1.0;
}
