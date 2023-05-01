#include "Helper.h"

vx_point_cloud_t* Helper::Voxelize(std::vector<vec3> vertices, std::vector<int> indices, float voxelsizex, float voxelsizey, float voxelsizez, float precision)
{
	vx_mesh_t* mesh;
	vx_point_cloud_t* voxelizedMesh;

	mesh = vx_mesh_alloc(vertices.size(), indices.size());

	for (size_t f = 0; f < indices.size(); f++) {
		mesh->indices[f] = indices[f];
	}

	for (size_t v = 0; v < vertices.size(); v++) {
		mesh->vertices[v].x = vertices[v].x;
		mesh->vertices[v].y = vertices[v].y;
		mesh->vertices[v].z = vertices[v].z;
	}

	voxelizedMesh = vx_voxelize_pc(mesh, voxelsizex, voxelsizey, voxelsizez, precision);

	printf("Number of vertices: %ld\n", voxelizedMesh->nvertices);
	//printf("Number of indices: %ld\n", voxelizedMesh->nindices);

	return voxelizedMesh;
}