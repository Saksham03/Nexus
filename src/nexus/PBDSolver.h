#pragma once

#include "Constraint.h"
#include "NexusObject.h"

class PBDSolver {
private:
	vec3 gravity;
	std::vector<uPtr<NexusObject>> objects;
	std::vector<uPtr<Constraint>> collConstraints;
public:
	PBDSolver();
	PBDSolver(vec3 gravity, std::vector<uPtr<NexusObject>> objects);
	~PBDSolver();
	void update(float deltaTime);

	void addObject(uPtr<NexusObject> obj);
	void precomputeConstraints();
	void generateCollisions();
	const std::vector<uPtr<NexusObject>>& getObjects() const;
};