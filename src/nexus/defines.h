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
#define NUM_SOLVER_SUBSTEPS 1