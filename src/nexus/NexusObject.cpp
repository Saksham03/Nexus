#include "NexusObject.h"
#include <stdio.h>

int NexusObject::lastId = 0;

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

int NexusObject::getObjectID()
{
	return NexusObject::lastId++;
}

void NexusObject::addStretchConstraint(Particle* p1, Particle* p2, float distance, float stiffness) {
	constraints.push_back(mkU<StretchConstraint>(p1, p2, distance, stiffness));
}

void NexusObject::fixParticle(Particle* p) {
	p->mass = -1.f;
	p->invMass = 0.f;
}