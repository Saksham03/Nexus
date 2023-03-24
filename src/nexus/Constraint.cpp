#include "Constraint.h"
#include "NexusCloth.h"
Constraint::Constraint()
	: stiffness(1.0f),
	equality(CONSTRAINT_EQUALITY_TYPE::EQUALITY)
{}

Constraint::Constraint(float stiffness, CONSTRAINT_EQUALITY_TYPE equality)
	: stiffness(stiffness),
	equality(equality)
{}

Constraint::~Constraint()
{
	/*NexusCloth(type, adsa);*/
}