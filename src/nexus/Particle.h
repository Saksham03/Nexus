#pragma once

#include "includes.h"

struct	Particle{
	/// <summary>
	/// Position
	/// </summary>
	vec3 x;		// position
	vec3 prevx;	// previous position
	/// <summary>
	/// Velocity
	/// </summary>
	vec3 v;		// velocity
	int phase;		// phase identifier
	float mass;			// mass
	float invMass;		// 1/mass

	Particle(float invMass)
		:x(), v(), phase(0), invMass(invMass)
	{}
	
	Particle(glm::vec3 pos, glm::vec3 vel, int p, float invMass)
		:x(pos), v(vel), phase(p), invMass(invMass)
	{}
};