#include "Constraint.h"
#include <algorithm>
#include <vector>
//#include <Eigen/src/Core00/Matrix.h>

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

// Function to compute the Jacobi rotation matrix corresponding to a Givens rotation.
mat3 JacobiRotation(vec2& u, vec2& v) {
	// Compute the sine and cosine of the angle of the rotation.
	double s = 2.0 * u[0] * v[1] - 2.0 * u[1] * v[0];
	double c = u[0] * u[0] + u[1] * u[1] - v[0] * v[0] - v[1] * v[1];

	// If the rotation is not a pure rotation, then we need to scale the vectors.
	if (c != 0.0) {
		double t = sqrt(s * s + c * c);
		u *= t / c;
		v *= t / c;
	}

	// Construct the rotation matrix.
	mat3 R;
	R[0][0] = c;
	R[0][1] = s;
	R[1][0] = -s;
	R[1][1] = c;
	return R;
}

// Function to decompose a 3x3 matrix into a rotation matrix using the Jacobi rotation method.
mat3 JacobiDecomposition(mat3& A) {
	// Initialize the rotation matrix to the identity matrix.
	mat3 R(1.0f);

	// Iterate until the matrix is diagonal.
	for (int iter = 0; iter < 10; iter++)
	{
		// Find the two columns of the matrix that have the largest off-diagonal element.
		int i = 0;
		int j = 1;
		for (int k = 0; k < 3; k++) {
			if (abs(A[i][k]) < abs(A[j][k])) {
				i = k;
			}
			else if (abs(A[i][k]) > abs(A[j][k])) {
				j = k;
			}
		}

		// Compute the Givens rotation that eliminates the off-diagonal element in column i.
		vec2 u = { A[i][i], A[i][j] };
		vec2 v = { A[j][i], A[j][j] };
		mat3 G = JacobiRotation(u, v);

		// Apply the Givens rotation to the matrix.
		A = G * A * glm::transpose(G);

		// If the matrix is now diagonal, then we are done.
		bool isDiag = true;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (i == j)
				{
					continue;
				}

				if (glm::abs(A[i][j] - A[j][i]) > 1e-10)
				{
					isDiag = false;
					break;
				}
			}

			if (isDiag == false)
			{
				break;
			}
		}

		if (isDiag)
		{
			break;
		}
	}

	// Return the rotation matrix.
	return R;
}

ShapeMatchingConstraint::ShapeMatchingConstraint(std::vector<uPtr<Particle>>* particles, float stiffness)
	: Constraint(stiffness, CONSTRAINT_TYPE::EQUALITY), particles(particles)
{
	// store rest configuration
	com_rest = vec3(0.0f);
	float sumMass = 0.0f;
	for (auto& particle : (*particles))
	{
		com_rest += particle->mass * particle->x;
		restPos.push_back(particle->x);
		sumMass += particle->mass;
	}
	com_rest /= sumMass;

	for (auto& particle : (*particles))
	{
		q.push_back(particle->x - com_rest);
	}
}

ShapeMatchingConstraint::~ShapeMatchingConstraint()
{}

//void extractRotation(const Matrix3d& A, Quaterniond& q,
//	const unsigned int maxIter)
//{
//	for (unsigned int iter = 0; iter < maxIter; iter++)
//	{
//		Matrix3d R = q.matrix();
//		Vector3d omega = (R.col(0).cross(A.col(0)) + R.col
//		(1).cross(A.col(1)) + R.col(2).cross(A.col(2))
//			) * (1.0 / fabs(R.col(0).dot(A.col(0)) + R.col
//			(1).dot(A.col(1)) + R.col(2).dot(A.col(2))) +
//				1.0e-9);
//		double w = omega.norm();
//		if (w < 1.0e-9)
//			break;
//		q = Quaterniond(AngleAxisd(w, (1.0 / w) * omega)) *
//			q;
//		q.normalize();
//	}
//}

void ShapeMatchingConstraint::projectConstraint()
{
	vec3 currCOM = getCurrentCOM();

	// covariance matrix A
	glm::mat3 A;
	for (int i = 0; i < particles->size(); i++)
	{
		Particle* particle = particles->at(i).get();
		vec3 p = particle->x - currCOM;

		A[0][0] += p[0] * q[i][0];	A[0][1] += p[0] * q[i][1];	A[0][2] += p[0] * q[i][2];
		A[1][0] += p[1] * q[i][0];	A[1][1] += p[1] * q[i][1];	A[1][2] += p[1] * q[i][2];
		A[2][0] += p[2] * q[i][0];	A[2][1] += p[2] * q[i][1];	A[0][2] += p[2] * q[i][2];
	}

	mat3 R = JacobiDecomposition(A);

	for (int i = 0; i < particles->size(); i++)
	{
		Particle* particle = particles->at(i).get();
		vec3 g = R * q[i] + currCOM;

		vec3 C = (g - particle->x) * stiffness;
		particle->x += C;
	}
}

vec3 ShapeMatchingConstraint::getCurrentCOM() const
{
	vec3 com = vec3(0.0);
	float sumMass = 0.0f;
	for (auto& particle : (*particles))
	{
		com += particle->mass * particle->x;
		sumMass += particle->mass;
	}
	com /= sumMass;
	return com;
}

#pragma endregion