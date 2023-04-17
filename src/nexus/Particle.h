#pragma once

#include "defines.h"

struct	Particle{
	/// <summary>
	/// Position
	/// </summary>
	vec3 x;		// position
	vec3 prevX;	// previous position
	/// <summary>
	/// Velocity
	/// </summary>
	vec3 v;		// velocity
	int phase;		// phase identifier
	float invMass;		// 1/mass

	Particle(float mass)
		:x(), v(), phase(0), invMass(mass < 0.0f ? 0.0f : 1.0f/mass), prevX()
	{}
	
	Particle(glm::vec3 pos, glm::vec3 vel, int p, float mass)
		:x(pos), v(vel), phase(p), invMass(mass < 0.0f ? 0.0f : 1.0f / mass), prevX()
	{}
};