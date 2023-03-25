#pragma once

#include <glm.hpp>
#include <memory>
#include <defines.h>

typedef glm::vec3 vec3;
typedef glm::vec2 vec2;
#define uPtr std::unique_ptr
#define mkU std::make_unique

// SOLVER RELATED DEFINES
#define GRAVITY vec3(0.0f, -9.8f, 0.0f)
#define NUM_SOLVER_SUBSTEPS 5

/// <summary>
/// Hashes two pointers and gives the same hash regardless of the ordering of the pointers.
/// </summary>
struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1*, T2*>& p) const {
        return std::hash<T1*>()(p.first) ^ std::hash<T2*>()(p.second);
    }
};