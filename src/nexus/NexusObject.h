#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <vector>

enum class NEXUS_OBJECT_TYPE { UNDEFINED, CLOTH, RIGIDBODY };

class NexusObject {
public:
	NEXUS_OBJECT_TYPE type;
	std::vector<Particle> particles;
	glm::vec3 force;

	NexusObject();
	NexusObject(NEXUS_OBJECT_TYPE, std::vector<Particle>, glm::vec3);
	~NexusObject();
	virtual void solve() = 0;
};