#include "Constraint.h"
#include "NexusCloth.h"
#include <iostream>

Constraint::Constraint()
	: stiffness(1.0f),
	equality(CONSTRAINT_EQUALITY_TYPE::EQUALITY)
{}

Constraint::Constraint(float stiffness, CONSTRAINT_EQUALITY_TYPE equality)
	: stiffness(stiffness),
	equality(equality)
{}

Constraint::~Constraint()
{}

DistanceConstraint::DistanceConstraint(float restLength, float stiffness)
	: restLength(restLength), Constraint(stiffness, CONSTRAINT_EQUALITY_TYPE::EQUALITY)
{}

float DistanceConstraint::projectConstraint(Particle* p[])
{
	vec3 dir = p[1]->x - p[0]->x;
	float currDist = glm::distance(p[1]->x, p[0]->x);

	float C = currDist - restLength;
	std::cout << currDist << "," << C << std::endl;
	if (std::abs(C) > 3.0f)
	{
		vec3 C1 = glm::normalize(dir);
		vec3 C2 = -glm::normalize(dir);
		float lambda = -C / (p[0]->invMass * 1 + p[1]->invMass * 1);

		vec3 delX1 = lambda * p[0]->invMass * C1;
		vec3 delX2 = lambda * p[1]->invMass * C2;

		p[0]->x += stiffness * delX1;
		p[1]->x += stiffness * delX2;
	}

	return 0.0f;
}
