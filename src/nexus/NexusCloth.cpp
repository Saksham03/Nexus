#include "NexusCloth.h"

NexusCloth::NexusCloth()
	: NexusCloth(std::vector<uPtr<Particle>>())
{}

NexusCloth::NexusCloth(std::vector<uPtr<Particle>> particles)
	: NexusObject(NEXUS_OBJECT_TYPE::CLOTH, std::move(particles))
{}

NexusCloth::~NexusCloth()
{}

void NexusCloth::update(float deltaTime)
{

}