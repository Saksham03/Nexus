#pragma once
#include <voxelizer.h>
#include <vector>

class Helper
{
private:
public:
	static vx_mesh_t* Voxelize(std::vector<float> vertices, std::vector<int> indices, float voxelsizex, float voxelsizey, float voxelsizez, float precision);
};