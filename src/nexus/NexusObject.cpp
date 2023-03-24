#include "NexusObject.h"
#include <stdio.h>

NexusObject::NexusObject()
	: type(NEXUS_OBJECT_TYPE::UNDEFINED)
{}

NexusObject::NexusObject(NEXUS_OBJECT_TYPE type, std::vector<uPtr<Particle>> particles)
	: type(type), particles(std::move(particles))
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
