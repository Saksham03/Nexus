#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>

class NexusCloth : public NexusObject {
public:
	NexusCloth();
	/// <summary>
	/// Nexus Cloth object
	/// </summary>
	/// <param name="particles">Set of particles that make up this cloth</param>
	NexusCloth(std::vector<uPtr<Particle>> particles);
	~NexusCloth();
	void update(float deltaTime) override;
};