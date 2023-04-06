#pragma once

#include "includes.h"

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
	float mass;
	float invMass;		// 1/mass
	float radius;
	vec3 color;

	Particle(float mass)
		: Particle(vec3(0.0f), vec3(0.0f), 0, mass)
	{}
	
	Particle(glm::vec3 pos, glm::vec3 vel, int p, float mass, float radius = 5.0f, vec3 color = vec3(1,0,0))
		:x(pos), 
		v(vel),
		phase(p), 
		mass(mass),
		invMass(mass < 0.0f ? 0.0f : 1.0f / mass), 
		prevX(), 
		radius(radius),
		color(color)
	{}
};