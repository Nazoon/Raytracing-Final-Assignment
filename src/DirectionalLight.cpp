#include "DirectionalLight.h"
#include <limits>

void DirectionalLight::direction(
	const Eigen::Vector3d& q, Eigen::Vector3d& d, double& max_t) const
{
	d = -(this->d.normalized());
	max_t = std::numeric_limits<double>::infinity();
}

Ray DirectionalLight::ray_to_target(const Eigen::Vector3d q) const {
	Ray r;
	double max_t;
	this->direction(q, r.direction, max_t);
	r.origin = q + r.direction;
	r.direction *= -1;
	r.cur_medium_refractive_index = 1.0;
	return r;
}