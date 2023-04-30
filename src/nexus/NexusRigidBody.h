#pragma once
#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>
#include <unordered_set>

class NexusRigidBody : public NexusObject {
private:
	float stiffness;

	float LENGTH, BREADTH, HEIGHT;	// we're assuming a cube rb only right now. TODO: allow arbitrary meshes
	mat4 currTransformMat;
public:
	NexusRigidBody();
	NexusRigidBody(float stiffness);
	/// <summary>
	/// Nexus RigidBody object
	/// </summary>
	/// <param name="particles">Set of particles that make up this RigidBody</param>
	NexusRigidBody(std::vector<uPtr<Particle>> particles, float stiffness);
	~NexusRigidBody();
	//void setLengthAndBreadth(int rows, int columns);
	void preComputeConstraints() override;
	const mat4& getCurrentTransformation() const;
};