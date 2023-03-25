#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <vector>

enum class NEXUS_OBJECT_TYPE { UNDEFINED, CLOTH, RIGIDBODY };

class NexusObject {
protected:
	std::vector<uPtr<Particle>> particles;
	std::vector<uPtr<Constraint>> constraints;

public:
	NEXUS_OBJECT_TYPE type;

	NexusObject();
	NexusObject(NEXUS_OBJECT_TYPE, std::vector<uPtr<Particle>>);
	~NexusObject();
	virtual void update(float deltaTime) = 0;

	void addParticle(uPtr<Particle> p);
	const std::vector<uPtr<Particle>>& getParticles() const;
	virtual void preComputeConstraints() = 0;
};