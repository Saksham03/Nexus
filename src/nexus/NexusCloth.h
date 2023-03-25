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

	std::unordered_set<std::pair<Particle*, Particle*>, PairHash> stretchConstraints;

	float nRows, nColumns;	// we're assuming a rectangular cloth only right now. TODO: allow arbitrary meshes

public:
	NexusCloth();
	NexusCloth(float stiffness, float restLength);
	/// <summary>
	/// Nexus Cloth object
	/// </summary>
	/// <param name="particles">Set of particles that make up this cloth</param>
	NexusCloth(std::vector<uPtr<Particle>> particles, float stiffness, float restLength);
	~NexusCloth();
	void setRowsAndColumns(int rows, int columns);
	void update(float deltaTime) override;
	void preComputeConstraints() override;
};