#include "NexusCloth.h"

NexusCloth::NexusCloth()
	: NexusCloth(std::vector<uPtr<Particle>>(), 0.8f, 0.5f)
{}

NexusCloth::NexusCloth(float stiffness, float restLength)
	: NexusCloth(std::vector<uPtr<Particle>>(), stiffness, restLength)
{}

NexusCloth::NexusCloth(std::vector<uPtr<Particle>> particles, float stiffness, float restLength)
	: NexusObject(NEXUS_OBJECT_TYPE::CLOTH, std::move(particles)), stiffness(stiffness), restLength(restLength)
{}

NexusCloth::~NexusCloth()
{}

void NexusCloth::update(float deltaTime)
{
	DistanceConstraint c(stiffness, restLength);
	for (int i = 0; i < particles.size() - 1; i++)
	{
		c.projectConstraint(new Particle * [] {particles[i].get(), particles[i+1].get()}, deltaTime);

		for (int j = i+1; j < particles.size(); j++)
		{
			c.projectConstraint(new Particle * [] {particles[i].get(), particles[j].get()}, deltaTime);
		}
	}
}

void NexusCloth::preComputeConstraints()
{
	
}
