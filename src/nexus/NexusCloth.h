#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>

class NexusCloth : public NexusObject {
private:
	float stiffness;
	float restLength;

public:
	NexusCloth();
	NexusCloth(float stiffness, float restLength);
	/// <summary>
	/// Nexus Cloth object
	/// </summary>
	/// <param name="particles">Set of particles that make up this cloth</param>
	NexusCloth(std::vector<uPtr<Particle>> particles, float stiffness, float restLength);
	~NexusCloth();
	void update(float deltaTime) override;
	void preComputeConstraints() override;
};