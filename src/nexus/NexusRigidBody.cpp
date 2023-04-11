#include "NexusRigidBody.h"

NexusRigidBody::NexusRigidBody()
	: NexusRigidBody(std::vector<uPtr<Particle>>(), 1.0f)
{}

NexusRigidBody::NexusRigidBody(float stiffness)
	: NexusRigidBody(std::vector<uPtr<Particle>>(), stiffness)
{}

NexusRigidBody::NexusRigidBody(std::vector<uPtr<Particle>> particles, float stiffness)
	: NexusObject(NEXUS_OBJECT_TYPE::RIGIDBODY, std::move(particles)), stiffness(stiffness),
	LENGTH(0), BREADTH(0), HEIGHT(0)
{}

NexusRigidBody::~NexusRigidBody()
{}

void NexusRigidBody::preComputeConstraints()
{	
	constraints.push_back(std::move(mkU<ShapeMatchingConstraint>(&particles)));
}
