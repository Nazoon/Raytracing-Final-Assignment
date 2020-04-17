#include "Object.h"
#include "Camera.h"
#include "Light.h"
#include "read_json.h"
#include "write_ppm.h"
#include "viewing_ray.h"
#include "raycolor.h"
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <memory>
#include <limits>
#include <functional>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <random>

#define _USE_MATH_DEFINES
#include <math.h>

const int rays_per_dim = 50;
const double min_t = 0.001;
const double skew = 0.01;

void setup_light_map(
	std::vector< std::shared_ptr<Object> > objects,
	std::vector< std::shared_ptr<Light> > lights,
	Eigen::Vector3d min,
	Eigen::Vector3d max,
	std::vector<LightPoint>& light_map
) {

	// We use a random skewing of each light ray to prevent banding
	// https://stackoverflow.com/questions/1340729/how-do-you-generate-a-random-double-uniformly-distributed-between-0-and-1-from-c/1340762
	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(-1, 1);

	Ray light_ray;
	Eigen::Vector3d ray_target;
	// For each light in the scene...
	for (std::shared_ptr<Light> l : lights) {
		// ...Point a ray of light from the source to a grid of points in the bounding box
		for (int x = 0; x < rays_per_dim; x++) {
			for (int y = 0; y < rays_per_dim; y++) {
				for (int z = 0; z < rays_per_dim; z++) {

					// Pointing a ray at the current position in the bounding box
					ray_target[0] = ((max[0] - min[0]) / rays_per_dim) * x + min[0];
					ray_target[1] = ((max[1] - min[1]) / rays_per_dim) * y + min[1];
					ray_target[2] = ((max[2] - min[2]) / rays_per_dim) * z + min[2];

					// Random skewing
					for (int i = 0; i < 3; i++) {
						double off = dist(e2);
						//printf("%f\n", off);
						ray_target[i] += off * skew;
					}

					//printf("--- casting ray (%d, %d, %d)...\n", x, y, z);

					light_ray = l->ray_to_target(ray_target);
					//cast_light(light_ray, l->I / (rays_per_dim * rays_per_dim * rays_per_dim), min_t, objects, 0, light_map);
					//cast_light(light_ray, l->I / (rays_per_dim * rays_per_dim), min_t, objects, 0, light_map);
					cast_light(light_ray, l->I / (rays_per_dim), min_t, objects, 0, light_map);
					//cast_light(light_ray, l->I , min_t, objects, 0, light_map);
				}
			}
		}
	}
}

const double frames_per_rotation = 360;
const double rad_per_frame = (M_PI * 2) / frames_per_rotation;
const double radius = 1.5;
const int bounces_per_rotation = 3;
const double max_height = 0.5;

Eigen::Vector3d sphere_pos(
	double rad
) {
	Eigen::Vector3d pos(0, 0, 0);

	// Rotation around the origin
	pos[0] = radius * std::cos(rad);
	pos[2] = radius * std::sin(rad);

	// Bouncing up and down :)
	pos[1] = std::abs((max_height + 0.5) * std::sin(rad * bounces_per_rotation)) - 0.5;

	return pos;
}

/*
For the dancing spheres
*/
void get_sphere_positions(
	int frame,
	Eigen::MatrixXd& ps) {

	int num_spheres = ps.rows();
	double rad_per_sphere = (M_PI * 2) / num_spheres;
	for (int i = 0; i < num_spheres; i++) {
		double rad = frame * rad_per_frame;
		rad += i * rad_per_sphere;
		ps.row(i) = sphere_pos(rad);
	}
}

int main(int argc, char* argv[])
{

	Eigen::Vector3d move_direction(0.05, 0.05, 0.0);
	int num_frames = 120;
	Camera camera;
	std::vector< std::shared_ptr<Object> > objects;
	std::vector< std::shared_ptr<Light> > lights;

	// Read a camera and scene description from given .json file
	std::string json_file = argv[1];
	int width = atoi(argv[2]);
	int height = atoi(argv[3]);
	read_json(
		json_file,
		camera,
		objects,
		lights);

	// Figuring out names for each frame so that they are processed in alphabetical order
	std::vector<std::string> names;
	for (int i = 0; i < num_frames; i++) {
		names.emplace_back("rgb" + std::to_string(i));
	}
	std::sort(names.begin(), names.end());

	// Rendering each frame
	std::vector<unsigned char> rgb_image(3 * width * height);
	for (int frame = 0; frame < num_frames; frame++) {
		//printf("- Frame %d/%d...\n", frame, num_frames);

		// Positioning spheres
		int num_spheres = 6;
		Eigen::MatrixXd sphere_pos;
		sphere_pos.resize(num_spheres, 3);
		get_sphere_positions(frame, sphere_pos);
		for (int i = 0; i < num_spheres; i++) {
			objects[i]->center = sphere_pos.row(i);
		}

		// Setting up bounding box for scene
		Eigen::Vector3d min(infinity, infinity, infinity);
		Eigen::Vector3d max = -min;
		for (int i = 0; i < objects.size(); i++) {
			Eigen::Vector3d obj_min, obj_max;
			if (objects[i]->bounding_corners(obj_min, obj_max)) {
				insert_point_into_box(min, max, obj_min);
				insert_point_into_box(min, max, obj_max);
			}
		}

		// Setting up light map for scene
		std::vector<LightPoint> light_map = std::vector<LightPoint>();/*
		printf("-- Setting up light map...\n");*/
		setup_light_map(objects, lights, min, max, light_map);
		//printf("--- # light rays cast = %d\n", rays_per_dim * rays_per_dim * rays_per_dim * (int)lights.size());
		//printf("--- # caustic points  = %d\n", (int)light_map.size());

		// Turning light map into KD tree
		//printf("-- Constructing KD tree...\n");
		std::shared_ptr<KDTree> light_map_tree = std::shared_ptr<KDTree>(new KDTree(light_map));/*
		printf("--- # caustic points  = %d\n", light_map_tree->num_points());
		printf("--- max depth = %d\n", light_map_tree->max_depth());*/

		assert(light_map.size() == light_map_tree->num_points());

		//printf("-- Drawing frame...\n");
		for (unsigned i = 0; i < height; ++i)
		{
			for (unsigned j = 0; j < width; ++j)
			{
				//printf("--- rendering (%d, %d)...\n", i, j);

				// Set background color
				Eigen::Vector3d rgb(0, 0, 0);

				// Compute viewing ray
				Ray ray;
				viewing_ray(camera, i, j, width, height, ray);

				// Shoot ray and collect color
				raycolor(ray, min_t, objects, lights, 0, light_map_tree, rgb);

				// Write double precision color into image
				auto clamp = [](double s) { return std::max(std::min(s, 1.0), 0.0); };
				rgb_image[0 + 3 * (j + width * i)] = 255.0 * clamp(rgb(0));
				rgb_image[1 + 3 * (j + width * i)] = 255.0 * clamp(rgb(1));
				rgb_image[2 + 3 * (j + width * i)] = 255.0 * clamp(rgb(2));

			}
		}
		write_ppm("frames/" + names[frame] + ".ppm", rgb_image, width, height, 3);
		objects[0]->center += move_direction;
		//printf("-- Frame done.\n");
	}
}
