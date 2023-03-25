#include "NexusObject.h"
#include <stdio.h>

NexusObject::NexusObject(NEXUS_OBJECT_TYPE type, std::vector<Particle> particles, glm::vec3 f)
	: type(type), particles(particles), force(f)
{}

NexusObject::NexusObject(NEXUS_OBJECT_TYPE type, std::vector<uPtr<Particle>> particles)
	: type(type), particles(std::move(particles)), constraints(std::vector<uPtr<Constraint>>())
{}

NexusObject::~NexusObject()
{}

void NexusObject::addParticle(uPtr<Particle> p)
{
	particles.push_back(std::move(p));
}

const std::vector<uPtr<Particle>>& NexusObject::getParticles() const
{
	return particles;
}
