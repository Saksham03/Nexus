#pragma once
#include <tiny_obj_loader.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "drawable.h"
#include <voxelizer.h>
#include "defines.h"
#include <memory>

// Only support triangle mesh now
class ObjModel
{
public:
	ObjModel();

	~ObjModel();

	// This function should be called after the initialization of OpenGL
	bool loadObj(const std::string& filename);
	bool loadObj(uPtr<vx_mesh_t> voxelizedMesh);
	void drawObj();

private:
	std::vector<tinyobj::shape_t> shapes; 
	std::vector<tinyobj::material_t> materials;
	tinyobj::attrib_t attrib;

	std::vector<std::unique_ptr<Drawable>> mDrawables;
};