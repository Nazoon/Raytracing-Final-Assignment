#include "write_ppm.h"
#include <fstream>
#include <cassert>
#include <iostream>
#include <string>

using namespace std;

bool write_ppm(
	const std::string& filename,
	const std::vector<unsigned char>& data,
	const int width,
	const int height,
	const int num_channels)
{
	assert(
		(num_channels == 3 || num_channels == 1) &&
		".ppm only supports RGB or grayscale images");

	ofstream out_file;
	out_file.open(filename, ios::out);

	// Magic number
	out_file << "P6" << endl;

	// Dimensions
	out_file << to_string(width) << endl;
	out_file << to_string(height) << endl;

	// Maxval
	out_file << "255" << endl;

	// Pixel values for each channel
	for (int row = 0; row < height; row++) {
		for (int col = 0; col < width; col++) {
			if (num_channels != 1) {
				for (int channel = 0; channel < num_channels; channel++) {
					out_file << data[channel + num_channels * (col + (width * row))];
				}
			}
			else {
				for (int channel = 0; channel < 3; channel++) {
					out_file << data[num_channels * (col + (width * row))];
				}
			}
		}
	}

	out_file.close();

	return true;
}
