#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>
#include <unordered_set>

class NexusCloth : public NexusObject {
private:
	float stiffness;
	float restLength;

	float LENGTH, BREADTH;	// we're assuming a rectangular cloth only right now. TODO: allow arbitrary meshes

public:
	NexusCloth();
	NexusCloth(float stiffness);
	/// <summary>
	/// Nexus Cloth object
	/// </summary>
	/// <param name="particles">Set of particles that make up this cloth</param>
	NexusCloth(std::vector<uPtr<Particle>> particles, float stiffness);
	~NexusCloth();
	void setLengthAndBreadth(int rows, int columns);
	void preComputeConstraints() override;
};