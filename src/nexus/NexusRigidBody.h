#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>
#include <unordered_set>

class NexusRigidBody : public NexusObject {
private:
	float stiffness;

	mat4 currTransformMat;

	std::vector<vec3> q;		// q vectors for this RB's vertex positions
	std::vector<vec3> originalVertexPositions;

	ShapeMatchingConstraint* smC;

public:
	NexusRigidBody();
	NexusRigidBody(float stiffness);
	/// <summary>
	/// Nexus RigidBody object
	/// </summary>
	/// <param name="particles">Set of particles that make up this RigidBody</param>
	NexusRigidBody(std::vector<uPtr<Particle>> particles, float stiffness);
	~NexusRigidBody();
	void setOriginalVerts(std::vector<vec3> originalVertexPositions);
	std::vector<vec3> getMovedVertices() const;
	void preComputeConstraints() override;
	const mat4& getCurrentTransformation() const;
};