#pragma once

#include "Constraint.h"
#include "NexusObject.h"
#include "SpatialHash.h"
#include <unordered_map>

class PBDSolver {
private:
	vec3 gravity;
	std::vector<uPtr<NexusObject>> objects;
	std::vector<uPtr<Constraint>> collConstraints;
	SpatialHash particleHash;

	uPtr<Particle> pBigBoi;
	
public:
	PBDSolver();
	PBDSolver(vec3 gravity, std::vector<uPtr<NexusObject>> objects);
	~PBDSolver();
	void update(float deltaTime);

	void addObject(uPtr<NexusObject> obj);
	void precomputeConstraints();
	void generateSpatialHash();
	void generateCollisions();
	const std::vector<uPtr<NexusObject>>& getObjects() const;
	void setBigBoi(uPtr<Particle>&& p);
};