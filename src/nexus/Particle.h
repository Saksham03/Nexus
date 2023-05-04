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
	double mass;
	double invMass;		// 1/mass
	double radius;
	vec3 color;

	Particle(){}
	Particle(double mass)
		: Particle(vec3(0.0f), vec3(0.0f), 0, mass)
	{}
	
	Particle(glm::vec3 pos, glm::vec3 vel, int p, double mass, double radius = 5.0, vec3 color = vec3(1,0,0))
		:x(pos), 
		v(vel),
		phase(p), 
		mass(mass),
		invMass(mass < 0.0f ? 0.0f : 1.0f / mass), 
		prevX(pos), 
		radius(radius),
		color(color)
	{}
};