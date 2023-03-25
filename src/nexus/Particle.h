#pragma once

#include <glm.hpp>

struct	Particle{
	/// <summary>
	/// Position
	/// </summary>
	glm::vec3 x;		// position
	glm::vec3 prevx;
	/// <summary>
	/// Velocity
	/// </summary>
	glm::vec3 v;		// velocity
	int phase;		// phase identifier
	float invMass;		// 1/mass

	Particle(float m_inv)
		:x(), v(), phase(0), invMass(m_inv)
	{}
	
	Particle(glm::vec3 pos, glm::vec3 vel, int p, float minv)
		:x(pos), v(vel), phase(p), invMass(minv)
	{}
};