#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>
#include <unordered_set>

class NexusCloth : public NexusObject {
private:
	double stiffness;
	double restLength;

	double LENGTH, BREADTH;	// we're assuming a rectangular cloth only right now. TODO: allow arbitrary meshes

public:
	NexusCloth();
	NexusCloth(double stiffness);
	/// <summary>
	/// Nexus Cloth object
	/// </summary>
	/// <param name="particles">Set of particles that make up this cloth</param>
	NexusCloth(std::vector<uPtr<Particle>> particles, double stiffness);
	~NexusCloth();
	void setLengthAndBreadth(int rows, int columns);
	void preComputeConstraints() override;
};