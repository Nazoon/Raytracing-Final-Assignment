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


int main(int argc, char* argv[])
{

	Eigen::Vector3d move_direction(0.05, 0.05, 0.0);
	int num_frames = 180;

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
	std::vector<unsigned char> rgb_image(3 * width * height);
	std::vector<std::string> names;
	for (int i = 0; i < num_frames; i++) {
		names.emplace_back("rgb" + std::to_string(i));
	}
	std::sort(names.begin(), names.end());
	for (int frame = 0; frame < num_frames; frame++) {
		// For each pixel (i,j)
		for (unsigned i = 0; i < height; ++i)
		{
			for (unsigned j = 0; j < width; ++j)
			{
				// Set background color
				Eigen::Vector3d rgb(0, 0, 0);

				// Compute viewing ray
				Ray ray;
				viewing_ray(camera, i, j, width, height, ray);

				// Shoot ray and collect color
				raycolor(ray, 1.0, objects, lights, 0, rgb);

				// Write double precision color into image
				auto clamp = [](double s) { return std::max(std::min(s, 1.0), 0.0); };
				rgb_image[0 + 3 * (j + width * i)] = 255.0 * clamp(rgb(0));
				rgb_image[1 + 3 * (j + width * i)] = 255.0 * clamp(rgb(1));
				rgb_image[2 + 3 * (j + width * i)] = 255.0 * clamp(rgb(2));

			}
		}
		write_ppm("frames/" + names[frame] + ".ppm", rgb_image, width, height, 3);
		objects[4]->center += move_direction;
	}
}
