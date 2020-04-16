#include "Sphere.h"
#include "Ray.h"
#include <Eigen/Core>
#include <math.h>
bool Sphere::intersect(
	const Ray& ray, const double min_t, double& t, Eigen::Vector3d& n) const
{
	Eigen::Vector3d oc = ray.origin - center;
	double a = ray.direction.dot(ray.direction);
	double b = 2 * oc.dot(ray.direction);
	double c = oc.dot(oc) - (radius * radius);
	double d = (b * b) - (4 * a * c);
	if (d < 0) {
		return false;
	}
	double ret_t = (-b - sqrt(d)) / (2 * a);
	if (ret_t >= min_t) {
		t = ret_t;
		Eigen::Vector3d intersection = t * ray.direction + ray.origin;
		n = intersection - center;
		n.normalize();
		return true;
	}
	return false;
}

bool Sphere::bounding_corners(Eigen::Vector3d& min, Eigen::Vector3d& max) const {
	for (int i = 0; i < 3; i++) {
		min[i] = this->center[i] - this->radius;
		max[i] = this->center[i] + this->radius;
	}
	return true;
}