#include "NexusRigidBody.h"

NexusRigidBody::NexusRigidBody()
	: NexusRigidBody(std::vector<uPtr<Particle>>(), 1.0f)
{}

NexusRigidBody::NexusRigidBody(float stiffness)
	: NexusRigidBody(std::vector<uPtr<Particle>>(), stiffness)
{}

NexusRigidBody::NexusRigidBody(std::vector<uPtr<Particle>> particles, float stiffness)
	: NexusObject(NEXUS_OBJECT_TYPE::RIGIDBODY, std::move(particles)), stiffness(stiffness),
	originalVertexPositions()
{}

NexusRigidBody::~NexusRigidBody()
{}

void NexusRigidBody::preComputeConstraints()
{	
	std::vector<Particle*> ps;
	for (auto& p : particles)
	{
		ps.push_back(p.get());
	}

	uPtr<ShapeMatchingConstraint> smc = mkU<ShapeMatchingConstraint>(ps, stiffness);

	for (int i = 0; i < originalVertexPositions.size(); i++)
	{
		q.push_back(originalVertexPositions[i] - smc->getCurrentCOM());
	}

	smC = smc.get();
	constraints.push_back(std::move(smc));
}

const mat4& NexusRigidBody::getCurrentTransformation() const
{
	return currTransformMat;
}

void NexusRigidBody::setOriginalVerts(std::vector<vec3> originalVertexPositions)
{
	this->originalVertexPositions = originalVertexPositions;
}

std::vector<vec3> NexusRigidBody::getMovedVertices() const
{
	std::vector<vec3> moved;

	mat3 sm = smC->getShapeMatchingMatrix();
	vec3 com = smC->getCurrentCOM();

	for (int i = 0; i < originalVertexPositions.size(); i++)
	{
		vec3 g = sm * q[i] + com;
		moved.push_back(originalVertexPositions[i] + (g - originalVertexPositions[i]) * stiffness);
	}

	return moved;
}
