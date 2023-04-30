#include "Helper.h"

vx_mesh_t* Helper::Voxelize(std::vector<float> vertices, std::vector<int> indices, float voxelsizex, float voxelsizey, float voxelsizez, float precision)
{
	vx_mesh_t* mesh;
	vx_mesh_t* voxelizedMesh;

	mesh = vx_mesh_alloc(vertices.size(), indices.size());

	for (size_t f = 0; f < indices.size(); f++) {
		mesh->indices[f] = indices[f];
	}

	for (size_t v = 0; v < vertices.size() / 3; v++) {
		mesh->vertices[v].x = vertices[3 * v + 0];
		mesh->vertices[v].y = vertices[3 * v + 1];
		mesh->vertices[v].z = vertices[3 * v + 2];
	}

	voxelizedMesh = vx_voxelize(mesh, voxelsizex, voxelsizey, voxelsizez, precision);

	printf("Number of vertices: %ld\n", voxelizedMesh->nvertices);
	printf("Number of indices: %ld\n", voxelizedMesh->nindices);

	return voxelizedMesh;
}