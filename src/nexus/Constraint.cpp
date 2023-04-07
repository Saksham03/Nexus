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

		return;
		// TODO: fix friction
		vec3 delta = deltaX2 - deltaX1;
		vec3 proj = glm::dot(delta, dir) * dir;
		vec3 tangential = -(delta - proj);
		float tangentialDist = glm::length(tangential);

		float us = 0.8f;
		float uk = 0.2f;

		float staticFric1 = glm::length(deltaX1) * us;
		deltaX1 = (w1 / (w1 + w2)) * tangential;
		if (tangentialDist >= staticFric1 + 0.0001f)
		{
			float dynamicFric = glm::length(deltaX1) * uk;
			deltaX1 *= std::min((dynamicFric / tangentialDist), 1.0f);
		}

		deltaX2 = -(w2 / (w1 + w2)) * deltaX1;

		p1->x += deltaX1;
		p2->x += deltaX2;
	}
}

bool ParticleParticleCollisionConstraint::areParticlesColliding(Particle* p1, Particle* p2)
{
	return (p1->radius + p2->radius > glm::length(p1->x - p2->x));
}

#pragma endregion