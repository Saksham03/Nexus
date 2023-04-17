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

DistanceConstraint::DistanceConstraint(Particle* p, glm::vec3 ref, float r)
	:Constraint(1.f, CONSTRAINT_TYPE::EQUALITY), p(p), ref(ref), radius(r)
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

StretchConstraint::StretchConstraint(Particle* p1, Particle* p2, float t)
	:Constraint(1.f, CONSTRAINT_TYPE::EQUALITY), p1(p1), p2(p2), threshold(t)
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

BendingConstraint::BendingConstraint(Particle* p1, Particle* p2, Particle* p3, Particle* p4, float t)
	:Constraint(0.7f, CONSTRAINT_TYPE::EQUALITY), p1(p1), p2(p2), p3(p3), p4(p4), threshold(t)
{}

BendingConstraint::~BendingConstraint()
{}

void BendingConstraint::projectConstraint() {
	vec3 P2 = p2->x - p1->x;
	vec3 P3 = p3->x - p1->x;
	vec3 P4 = p4->x - p1->x;

	vec3 n1 = glm::normalize(glm::cross(P2, P3));
	vec3 n2 = glm::normalize(glm::cross(P2, P4));
	float d = glm::clamp(glm::dot(n1, n2), -1.f, 1.f);
	float d1 = glm::dot(n1, n2);
	function = glm::acos(d) - threshold;
	if (!isConstraintSatisfied()) {
		vec3 q3 = (glm::cross(P2, n2) + d * (glm::cross(n1, P2))) / glm::length(glm::cross(P2, P3));
		vec3 q4 = (glm::cross(P2, n1) + d * (glm::cross(n2, P2))) / glm::length(glm::cross(P2, P4));
		vec3 q2 =
			-(glm::cross(P3, n2) + d * (glm::cross(n1, P3))) / glm::length(glm::cross(P2, P3))
			-(glm::cross(P4, n1) + d * (glm::cross(n2, P4))) / glm::length(glm::cross(P2, P4));
		vec3 q1 = -q2 - q3 - q4;

		float weightFactor =
			p1->invMass * glm::dot(q1, q1) +
			p2->invMass * glm::dot(q2, q2) +
			p3->invMass * glm::dot(q3, q3) +
			p4->invMass * glm::dot(q4, q4);

		float wSum = p1->invMass + p2->invMass + p3->invMass + p4->invMass;
		float qSum = glm::dot(q1, q1) + glm::dot(q2, q2) + glm::dot(q3, q3) + glm::dot(q4, q4);

		float mulFactor = sqrt(1.f - d * d) * function;
		
		weightFactor = wSum * qSum;

		vec3 deltaX1 = -p1->invMass * mulFactor * q1 / weightFactor;
		vec3 deltaX2 = -p2->invMass * mulFactor * q2 / weightFactor;
		vec3 deltaX3 = -p3->invMass * mulFactor * q3 / weightFactor;
		vec3 deltaX4 = -p4->invMass * mulFactor * q4 / weightFactor;
		
		p1->x += stiffness * deltaX1;
		p2->x += stiffness * deltaX2;
		p3->x += stiffness * deltaX3;
		p4->x += stiffness * deltaX4;
	}
}