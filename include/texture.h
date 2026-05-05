#pragma once

#include <iostream>
#include "stb_image.h"

GLuint setup_texture(const char* filename)
{
	//enable textures
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);

	//generate OpenGL texture object
	GLuint texObject;
	glGenTextures(1, &texObject);
	glBindTexture(GL_TEXTURE_2D, texObject);

	//some params for how OpenGL will draw the texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	//for smoother image use GL_LINEAR
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	//load the image
	int w, h, chan;
	// need to flip the y-axis of the image
	stbi_set_flip_vertically_on_load(true);

	//w h and chan will be written to after the function runs
	unsigned char* pxls = stbi_load(filename, &w, &h, &chan, 0);
	

	

	if (pxls)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, pxls);
	}

	glGenerateMipmap(GL_TEXTURE_2D);

	delete[] pxls;

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);

	return texObject;
}

GLuint setup_mipmaps(const char* filename[], int n)
{
	//enable textures
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);

	//generate OpenGL texture object
	GLuint texObject;
	glGenTextures(1, &texObject);
	glBindTexture(GL_TEXTURE_2D, texObject);

	//some params for how OpenGL will draw the texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	//for smoother image use GL_LINEAR
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//set minification filter
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

	//load the image
	int w[16], h[16], chan[16];
	unsigned char* pxls[16];
	// need to flip the y-axis of the image
	stbi_set_flip_vertically_on_load(true);

	//w h and chan will be written to after the function runs
	for (int c = 0; c < n; c++)
	{
		//address of where the cth element is stored for w h and chan
		pxls[c] = stbi_load(filename[c], &w[c], &h[c], &chan[c], 0);
		if (pxls[c])
		{
			glTexImage2D(GL_TEXTURE_2D, c, GL_RGB, w[c], h[c], 0, GL_RGB, GL_UNSIGNED_BYTE, pxls[c]);
		}
		delete pxls[c];
		

	}


	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);

	return texObject;
}