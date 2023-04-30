#pragma once

#include "Constraint.h"
#include "NexusObject.h"
#include "SpatialHash.h"
#include <unordered_map>
#include <unordered_set>

class PBDSolver {
private:
	vec3 gravity;
	std::vector<uPtr<NexusObject>> objects;
	std::vector<uPtr<Constraint>> collConstraints;
	std::unordered_set<std::pair<Particle*, Particle*>, PairHash> alreadyCheckedCollisions;

	SpatialHash particleHash;
	
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
};