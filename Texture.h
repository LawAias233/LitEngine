#pragma once

#include <iostream>

class Texture
{
public:
	unsigned char* data;
	int w, h, n;
public:
	//char const *filename, int *x, int *y, int *channels_in_file, int desired_channels)
	Texture(const char* filename);

	~Texture();
};

