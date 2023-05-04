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

void NexusObject::addStretchConstraint(Particle* p1, Particle* p2, double distance, double stiffness) {
	constraints.push_back(mkU<StretchConstraint>(p1, p2, distance, stiffness));
}

void NexusObject::addBendingConstraint(Particle* p1, Particle* p2, Particle* p3, Particle* p4, double stiffness) {
	vec3 n1 = glm::normalize(glm::cross(p2->x - p1->x, p3->x - p1->x));
	vec3 n2 = glm::normalize(glm::cross(p2->x - p1->x, p4->x - p1->x));
	uPtr<BendingConstraint> d = mkU<BendingConstraint>(p1, p2, p3, p4, glm::acos(glm::clamp(glm::dot(n1, n2), -1.0, 1.0)), stiffness);
	constraints.push_back(std::move(d));
}

void NexusObject::fixParticle(Particle* p) {

	std::unique_ptr<DistanceConstraint> d1 = std::make_unique<DistanceConstraint>(p, p->x, 0.f);
	constraints.push_back(std::move(d1));
}