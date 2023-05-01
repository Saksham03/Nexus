#pragma once
#include <voxelizer.h>
#include "defines.h"
#include <vector>

class Helper
{
private:
public:
	static vx_point_cloud_t* Voxelize(std::vector<vec3> vertices, std::vector<int> indices, float voxelsizex, float voxelsizey, float voxelsizez, float precision);
};