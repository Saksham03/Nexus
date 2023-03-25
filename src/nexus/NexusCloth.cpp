#include "NexusCloth.h"

NexusCloth::NexusCloth()
	: NexusObject()
{}

NexusCloth::NexusCloth(NEXUS_OBJECT_TYPE type, std::vector<Particle> particles)
	: NexusObject(type, particles, glm::vec3())
{}

NexusCloth::~NexusCloth()
{}

void solve()
{

}