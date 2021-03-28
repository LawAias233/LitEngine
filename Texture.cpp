#include "Texture.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

Texture::Texture(const char* filename)
	: data(nullptr), w(0), h(0), n(0)
{
	data = stbi_load(filename, &w, &h, &n, 0);
	if (data == nullptr)
	{
		std::cerr << "cant load texture:" << filename << std::endl;
		exit(-1);
	}
}

Texture::~Texture()
{
	stbi_image_free(data);
}
