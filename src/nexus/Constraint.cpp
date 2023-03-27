#include "Constraint.h"
#include "NexusCloth.h"
Constraint::Constraint()
	: stiffness(1.0f),
	type(CONSTRAINT_TYPE::EQUALITY),
	function(-1.f)
{}

Constraint::Constraint(float stiffness, CONSTRAINT_TYPE type)
	: stiffness(stiffness),
	type(type),
	function(-1.f)
{}

Constraint::~Constraint()
{
}

bool Constraint::isConstraintSatisfied() {
	switch (type) {
	case(CONSTRAINT_TYPE::EQUALITY): return function == 0.f;
	case(CONSTRAINT_TYPE::INEQUALITY): return function >= 0.f;
	default: return true;
	}
}

DistanceConstraint::DistanceConstraint(Particle* p, glm::vec3 ref, float r, float stiffness)
	:Constraint(stiffness, CONSTRAINT_TYPE::EQUALITY), p(p), ref(ref), radius(r)
{}

DistanceConstraint::~DistanceConstraint()
{}

void DistanceConstraint::projectConstraint() {
	float dist = glm::distance(p->x, ref);
	function = dist - radius;
	if (!isConstraintSatisfied()) {
		glm::vec3 C = glm::normalize(p->x - ref);
		glm::vec3 deltaX = -function * C;
		p->x += stiffness * deltaX;
	}
}

StretchConstraint::StretchConstraint(Particle* p1, Particle* p2, float t, float stiffness)
	:Constraint(stiffness, CONSTRAINT_TYPE::EQUALITY), p1(p1), p2(p2), threshold(t)
{}

StretchConstraint::~StretchConstraint()
{}

void StretchConstraint::projectConstraint() {
	float dist = glm::distance(p1->x, p2->x);	
	function = dist - threshold;
	if (!isConstraintSatisfied()) {
		glm::vec3 C1 = glm::normalize(p1->x - p2->x);
		glm::vec3 C2 = -C1;
		float w1 = p1->invMass;
		float w2 = p2->invMass;
		float lambda = -function / (w1 + w2);
		glm::vec3 deltaX1 = lambda * w1 * C1;
		glm::vec3 deltaX2 = lambda * w2 * C2;
		p1->x += stiffness * deltaX1;
		p2->x += stiffness * deltaX2;
	}
}

#pragma region Particle-Particle-Collisions
/* -------------------------------------------------------------------
------------------ PARTICLE-PARTICLE COLLISIONS ----------------------
---------------------------------------------------------------------- */

ParticleParticleCollisionConstraint::ParticleParticleCollisionConstraint(Particle* p1, Particle* p2, float stiffness)
	: Constraint(stiffness, CONSTRAINT_TYPE::INEQUALITY), p1(p1), p2(p2)
{}

ParticleParticleCollisionConstraint::~ParticleParticleCollisionConstraint()
{}

void ParticleParticleCollisionConstraint::projectConstraint()
{
	vec3 dir = p2->x - p1->x;
	float dist = glm::length(dir);

	if (dist > p1->radius + p2->radius)
	{
		// No collisions
		return;
	}

	function = dist - (p1->radius + p2->radius);	// should be > 0.0f

	if (!isConstraintSatisfied())
	{
		dir = glm::normalize(dir);

		float w1 = p1->invMass;
		float w2 = p2->invMass;
		float lambda = function / (w1 + w2);

		glm::vec3 C1 = dir;
		glm::vec3 C2 = -C1;

		glm::vec3 deltaX1 = lambda * w1 * C1;
		glm::vec3 deltaX2 = lambda * w2 * C2;

		p1->x += deltaX1 * stiffness;
		p2->x += deltaX2 * stiffness;

		//float v1 = glm::dot(p1->v, dir);
		//float v2 = glm::dot(p2->v, dir);

		//float m1 = p1->mass;
		//float m2 = p2->mass;
		//float newV1 = (m1 * v1 + m2 * v2 - m2 * (v1 - v2) * stiffness) / (m1 + m2);
		//float newV2 = (m1 * v1 + m2 * v2 - m1 * (v2 - v1) * stiffness) / (m1 + m2);

		//glm::vec3 deltaV1 = lambda * w1 * (newV1 - v1) * dir;
		//glm::vec3 deltaV2 = lambda * w1 * (newV2 - v2) * dir;

		/*p1->v += deltaV1;
		p2->v += deltaV2;*/
	}
}
#pragma endregion