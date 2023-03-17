#pragma once

#include "Particle.h"
#include <vector>

enum class CONSTRAINT_EQUALITY_TYPE 
{
    /// <summary>
    /// Satisfied when constraint == 0
    /// </summary>
    EQUALITY,
    /// <summary>
    /// Satisfied when constraint >= 0
    /// </summary>
    INEQUALITY
};

// Abstract Constraint class
class Constraint 
{
private:
    std::vector<int> indices;
    CONSTRAINT_EQUALITY_TYPE equality;
    float stiffness;
public:
    Constraint();
    Constraint(float, CONSTRAINT_EQUALITY_TYPE);
    virtual ~Constraint();
    virtual float projectConstraint(Particle* p) = 0;
};