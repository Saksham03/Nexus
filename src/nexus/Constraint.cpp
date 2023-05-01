#include "Constraint.h"

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
{}

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
		/*p1->color = vec3(1, 0, 0);
		p2->color = vec3(1, 0, 0);*/

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

		// TODO: fix friction
		vec3 relativeDisplacement = (p2->x - p2->prevX) - (p1->x - p1->prevX);
		//vec3 d = glm::normalize(p2->prevX - p1->prevX);
		vec3 proj = glm::dot(relativeDisplacement, dir) * dir;
		vec3 tangential = (relativeDisplacement - proj);
		float tangentialDist = glm::length(tangential);

		float us = 0.1f;
		float uk = 0.05f;

		float staticFric1 = glm::length(deltaX1) * us;
		deltaX1 = (w1 / (w1 + w2)) * tangential;
		float temp = glm::length(deltaX1);
		if (tangentialDist >= staticFric1 + 0.0001f)
		{
			float dynamicFric = glm::length(deltaX1) * uk;
			deltaX1 *= std::min((dynamicFric / tangentialDist), 1.0f);
		}

		deltaX2 = -(w2 / (w1 + w2)) * deltaX1;

		p1->x += deltaX1;
		p2->x += deltaX2;
	}
	else
	{
		/*p1->color = vec3(0, 1, 0);
		p2->color = vec3(0, 1, 0);*/
	}
}

bool ParticleParticleCollisionConstraint::areParticlesColliding(Particle* p1, Particle* p2)
{
	return (p1->radius + p2->radius > glm::length(p1->x - p2->x));
}

#pragma endregion

#pragma region Shape-Matching-Constraint
/* -------------------------------------------------------------------
------------------------- SHAPE-MATCHING -----------------------------
---------------------------------------------------------------------- */
ShapeMatchingConstraint::ShapeMatchingConstraint(std::vector<Particle*> particles, float stiffness)
	: Constraint(stiffness, CONSTRAINT_TYPE::EQUALITY), particles(particles), prevRot(Quaterniond::Identity()),
	shapeMatchingMat(mat3(1.0f))
{
	// store rest configuration
	updateCurrentCom();
	com_rest = getCurrentCOM();

	for (auto& particle : (particles))
	{
		q.push_back(particle->x - com_rest);
	}
}

ShapeMatchingConstraint::~ShapeMatchingConstraint()
{}

void ShapeMatchingConstraint::projectConstraint()
{
	updateCurrentCom();

	// covariance matrix A
	Matrix3d A;
	A.setZero();
	for (int i = 0; i < particles.size(); i++)
	{
		Particle* particle = particles[i];
		vec3 p = particle->x - currCOM;

		p *= particle->mass;

		A(0, 0) += p[0] * q[i][0];	A(0, 1) += p[0] * q[i][1];	A(0, 2) += p[0] * q[i][2];
		A(1, 0) += p[1] * q[i][0];	A(1, 1) += p[1] * q[i][1];	A(1, 2) += p[1] * q[i][2];
		A(2, 0) += p[2] * q[i][0];	A(2, 1) += p[2] * q[i][1];	A(2, 2) += p[2] * q[i][2];
	}

	extractRotation(A, prevRot, 2);
	Matrix3d R = prevRot.matrix();
	shapeMatchingMat = mat3();
	shapeMatchingMat[0][0] = R(0, 0);  shapeMatchingMat[0][1] = R(0, 1); shapeMatchingMat[0][2] = R(0, 2);
	shapeMatchingMat[1][0] = R(1, 0);  shapeMatchingMat[1][1] = R(1, 1); shapeMatchingMat[1][2] = R(1, 2);
	shapeMatchingMat[2][0] = R(2, 0);  shapeMatchingMat[2][1] = R(2, 1); shapeMatchingMat[2][2] = R(2, 2);
	shapeMatchingMat = glm::transpose(shapeMatchingMat);
	for (int i = 0; i < particles.size(); i++)
	{
		Particle* particle = particles[i];
		vec3 g = shapeMatchingMat * q[i] + currCOM;

		vec3 C = (g - particle->x) * stiffness;

		particle->x += C;
	}
}

void ShapeMatchingConstraint::updateCurrentCom()
{
	currCOM = vec3(0.0);
	float sumMass = 0.0f;
	for (auto& particle : particles)
	{
		currCOM += particle->mass * particle->x;
		sumMass += particle->mass;
	}
	currCOM /= sumMass;
}

void ShapeMatchingConstraint::extractRotation(const Matrix3d& A, Quaterniond& q, const unsigned int maxIter) const
{
	for (unsigned int iter = 0; iter < maxIter; iter++)
	{
		Matrix3d R = q.matrix();
		Vector3d omega = (R.col(0).cross(A.col(0)) + R.col
		(1).cross(A.col(1)) + R.col(2).cross(A.col(2))
			) * (1.0 / fabs(R.col(0).dot(A.col(0)) + R.col
			(1).dot(A.col(1)) + R.col(2).dot(A.col(2))) +
				1.0e-9);
		double w = omega.norm();
		if (w < 1.0e-9)
			break;
		q = Quaterniond(AngleAxisd(w, (1.0 / w) * omega)) *
			q;
		q.normalize();
	}
}

const mat3& ShapeMatchingConstraint::getShapeMatchingMatrix() const
{
	return shapeMatchingMat;
}

const vec3& ShapeMatchingConstraint::getCurrentCOM() const
{
	return currCOM;
}

#pragma endregion


BendingConstraint::BendingConstraint(Particle* p1, Particle* p2, Particle* p3, Particle* p4, float t)
	:Constraint(1.f, CONSTRAINT_TYPE::EQUALITY), p1(p1), p2(p2), p3(p3), p4(p4), threshold(t)
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
			- (glm::cross(P4, n1) + d * (glm::cross(n2, P4))) / glm::length(glm::cross(P2, P4));
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