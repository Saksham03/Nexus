#pragma once

#include <glm.hpp>

struct	Particle{
public:
	/// <summary>
	/// Position
	/// </summary>
	glm::vec3 x;		// position
	/// <summary>
	/// Velocity
	/// </summary>
	glm::vec3 v;		// velocity
	int phase;		// phase identifier
	float invMass;		// 1/mass
};