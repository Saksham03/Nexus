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
			Particle* p1 = particles[i * LENGTH + j].get();
			Particle* p2 = particles[i * LENGTH + j + 1].get();
			std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(p1, p2, glm::length(p2->x - p1->x), stiffness);
			constraints.push_back(std::move(d));
		}
	}
	for (int i = 0; i < BREADTH - 1; i++) {
		for (int j = 0; j < LENGTH; j++) {
			std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(particles[i * LENGTH + j].get(), particles[(i + 1) * LENGTH + j].get(), 2.f);
			constraints.push_back(std::move(d));
		}
	}
	//Bending constraints

	//across one diagonal
#define TOTAL 1
#define DIAG 1
#define VERT 1
#define HORZ 1

#if TOTAL
	#if DIAG
	for (int i = 1; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH - 1; j++) {
			Particle* p1 = particles[i * LENGTH + j].get();
			Particle* p2 = particles[(i - 1) * LENGTH + j + 1].get();
			Particle* p3 = particles[(i - 1) * LENGTH + j].get();
			Particle* p4 = particles[i * LENGTH + j + 1].get();
			vec3 n1 = glm::normalize(glm::cross(p2->x - p1->x, p3->x - p1->x));
			vec3 n2 = glm::normalize(glm::cross(p2->x - p1->x, p4->x - p1->x));
			uPtr<BendingConstraint> d = mkU<BendingConstraint>(p1, p2, p3, p4, glm::acos(glm::dot(n1, n2)));
			constraints.push_back(std::move(d));
		}
	}
	#endif

	//across a vertical edge
	#if VERT
	for (int i = 1; i < BREADTH; i++) {
		for (int j = 1; j < LENGTH - 1; j++) {
			Particle* p1 = particles[i * LENGTH + j].get();
			Particle* p2 = particles[(i - 1) * LENGTH + j].get();
			Particle* p3 = particles[i * LENGTH + j - 1].get();
			Particle* p4 = particles[(i - 1) * LENGTH + j + 1].get();
			vec3 n1 = glm::normalize(glm::cross(p2->x - p1->x, p3->x - p1->x));
			vec3 n2 = glm::normalize(glm::cross(p2->x - p1->x, p4->x - p1->x));
			uPtr<BendingConstraint> d = mkU<BendingConstraint>(p1, p2, p3, p4, glm::acos(glm::clamp(glm::dot(n1, n2), -1.f, 1.f)));
			constraints.push_back(std::move(d));
		}
	}
	#endif

	//across a horizontal edge
	#if HORZ
	for (int i = 1; i < BREADTH - 1; i++) {
		for (int j = 0; j < LENGTH - 1; j++) {
			Particle* p1 = particles[i * LENGTH + j].get();
			Particle* p2 = particles[i * LENGTH + j + 1].get();
			Particle* p3 = particles[(i - 1) * LENGTH + j + 1].get();
			Particle* p4 = particles[(i + 1) * LENGTH + j].get();
			vec3 n1 = glm::normalize(glm::cross(p2->x - p1->x, p3->x - p1->x));
			vec3 n2 = glm::normalize(glm::cross(p2->x - p1->x, p4->x - p1->x));
			uPtr<BendingConstraint> d = mkU<BendingConstraint>(p1, p2, p3, p4, glm::acos(glm::dot(n1, n2)));
			constraints.push_back(std::move(d));
		}
	}
	#endif
#endif

	/*std::unique_ptr<DistanceConstraint> d1 = std::make_unique<DistanceConstraint>(particles[0].get(), particles[0]->x, 0.f);
	constraints.push_back(std::move(d1));
	std::unique_ptr<DistanceConstraint> d2 = std::make_unique<DistanceConstraint>(particles[LENGTH - 1].get(), particles[LENGTH - 1]->x, 0.f);
	constraints.push_back(std::move(d2));*/
}
