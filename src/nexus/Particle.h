#pragma once

#include "includes.h"

typedef struct	Particle {
public:
	/// <summary>
	/// Position
	/// </summary>
	vec3 x;		// position
	/// <summary>
	/// Velocity
	/// </summary>
	vec3 v;		// velocity
	int phase;		// phase identifier
	float invMass;		// 1/mass

	Particle(vec3 position, vec3 velocity, float mass, int phase);
} Particle;