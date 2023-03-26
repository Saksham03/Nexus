#include "NexusCloth.h"

NexusCloth::NexusCloth()
	: NexusCloth(std::vector<uPtr<Particle>>(), 1.0f)
{}

NexusCloth::NexusCloth(float stiffness)
	: NexusCloth(std::vector<uPtr<Particle>>(), stiffness)
{}

NexusCloth::NexusCloth(std::vector<uPtr<Particle>> particles, float stiffness)
	: NexusObject(NEXUS_OBJECT_TYPE::CLOTH, std::move(particles)), stiffness(stiffness),
	LENGTH(0), BREADTH(0)
{}

NexusCloth::~NexusCloth()
{}

void NexusCloth::setLengthAndBreadth(int length, int breadth)
{
	LENGTH = length;
	BREADTH = breadth;
}

void NexusCloth::preComputeConstraints()
{
	// Stretch constraints
	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH - 1; j++) {
			std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(particles[i * LENGTH + j].get(), particles[i * LENGTH + j + 1].get(), 3.f);
			constraints.push_back(std::move(d));
		}
	}
	for (int i = 0; i < BREADTH - 1; i++) {
		for (int j = 0; j < LENGTH; j++) {
			std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(particles[i * LENGTH + j].get(), particles[(i + 1) * LENGTH + j].get(), 3.f);
			constraints.push_back(std::move(d));
		}
	}
	/*std::unique_ptr<DistanceConstraint> d1 = std::make_unique<DistanceConstraint>(particles[0].get(), particles[0]->x, 0.f);
	constraints.push_back(std::move(d1));
	std::unique_ptr<DistanceConstraint> d2 = std::make_unique<DistanceConstraint>(particles[LENGTH - 1].get(), particles[LENGTH - 1]->x, 0.f);
	constraints.push_back(std::move(d2));*/
}
