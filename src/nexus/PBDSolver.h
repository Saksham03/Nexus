#pragma once

#include "Constraint.h"
#include "NexusObject.h"
#include "SpatialHash.h"
#include <unordered_map>
#include <unordered_set>

struct SolverAttributes {
	int solverIterations;
	int solverSubsteps;
	vec3 gravity;
	vec3 wind;

	SolverAttributes(int iter, int substeps, vec3 gravity, vec3 wind)
	{
		this->solverIterations = iter;
		this->solverSubsteps = substeps;
		this->gravity = gravity;
		this->wind = wind;
	}
};

class PBDSolver {
private:
	SolverAttributes solverAttributes;
	std::vector<uPtr<NexusObject>> objects;
	std::vector<uPtr<Constraint>> collConstraints;
	std::unordered_set<std::pair<Particle*, Particle*>, PairHash> alreadyCheckedCollisions;

	SpatialHash particleHash;
	
public:
	PBDSolver();
	PBDSolver(std::vector<uPtr<NexusObject>> objects);
	~PBDSolver();
	void update(float deltaTime);

	void addObject(uPtr<NexusObject> obj);
	void setSolverAttributes(SolverAttributes sa);
	void precomputeConstraints();
	void generateSpatialHash();
	void generateCollisions();
	const std::vector<uPtr<NexusObject>>& getObjects() const;
};