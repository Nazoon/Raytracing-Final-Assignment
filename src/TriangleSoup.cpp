#include "TriangleSoup.h"
#include "Ray.h"
#include "first_hit.h"

bool TriangleSoup::intersect(
	const Ray& ray, const double min_t, double& t, Eigen::Vector3d& n) const
{
	int hit_id;
	return first_hit(ray, min_t, triangles, hit_id, t, n);
}

void TriangleSoup::bounding_corners(Eigen::Vector3d& min, Eigen::Vector3d& max) const {
	for (int i = 0; i < this->triangles.size(); i++) {
		this->triangles[i]->bounding_corners(min, max);
	}
}