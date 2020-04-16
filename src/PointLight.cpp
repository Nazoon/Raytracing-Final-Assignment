#include "PointLight.h"

void PointLight::direction(
	const Eigen::Vector3d& q, Eigen::Vector3d& d, double& max_t) const
{
	d = p - q;
	max_t = d.norm();
	d.normalize();
}

Ray PointLight::ray_to_target(const Eigen::Vector3d q) const {
	Ray r;
	double max_t;
	this->direction(q, r.direction, max_t);
	r.origin = this->p;
	r.direction *= -1;
	return r;
}