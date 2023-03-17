#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <vector>

enum class NEXUS_OBJECT_TYPE { UNDEFINED, CLOTH, RIGIDBODY };

class NexusObject {
public:
	NEXUS_OBJECT_TYPE type;
	std::vector<Particle> particles;

	NexusObject();
	NexusObject(NEXUS_OBJECT_TYPE, std::vector<Particle>);
	~NexusObject();
	virtual void solve() = 0;
};