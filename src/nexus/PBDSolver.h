#pragma once

#include "Constraint.h"
#include "NexusObject.h"

class PBDSolver {
private:
	vec3 gravity;
	std::vector<uPtr<NexusObject>> objects;
public:
	PBDSolver();
	PBDSolver(vec3 gravity, std::vector<uPtr<NexusObject>> objects);
	~PBDSolver();
	void update(float deltaTime);

	void addObject(uPtr<NexusObject> obj);
	const std::vector<uPtr<NexusObject>>& getObjects() const;
};