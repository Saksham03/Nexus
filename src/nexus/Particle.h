#pragma once

#include <glm.hpp>

struct Particle{
public:
	glm::vec3 x;		// position
	glm::vec3 v;		// velocity
	int phase;		// phase identifier
	float invMass;		// 1/mass
};