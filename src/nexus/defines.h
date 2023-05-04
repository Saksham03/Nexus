#pragma once

#include <glm.hpp>
#include <memory>

typedef glm::dvec3 vec3;
typedef glm::dvec2 vec2;
typedef glm::dmat3 mat3;
typedef glm::dmat4 mat4;
#define uPtr std::unique_ptr
#define mkU std::make_unique

// SOLVER RELATED DEFINES
#define GRAVITY vec3(0.0, -9.8, 0.0)
#define NUM_SOLVER_SUBSTEPS 1
#define NUM_SOLVER_ITERATIONS 5
#define FIXED_TIMESTEP 0.02    // roughly equivalent to 50 FPS, same as Unity
#define FIXED_PARTICLE_SIZE 0.25
#define SPATIAL_HASH_GRID_SIZE (4.0 * FIXED_PARTICLE_SIZE)

/// <summary>
/// Hashes two pointers and gives the same hash regardless of the ordering of the pointers.
/// </summary>
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1*, T2*>& p) const {
        return std::hash<T1*>()(p.first) ^ std::hash<T2*>()(p.second);
    }
};