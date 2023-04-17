#pragma once

#include "Particle.h"
#include <vector>

enum class CONSTRAINT_TYPE 
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
protected:
    std::vector<int> indices;
    CONSTRAINT_TYPE type;
    float stiffness;
    float function; //evaluation of the function at the current state

public:
    Constraint();
    Constraint(float, CONSTRAINT_TYPE);
    virtual ~Constraint();
    bool isConstraintSatisfied();
    virtual void projectConstraint() = 0;
};

class DistanceConstraint : public Constraint
{
private:
    Particle* p;
    glm::vec3 ref;
    float radius;

public:
    DistanceConstraint(Particle*, glm::vec3, float);
    ~DistanceConstraint();
    void projectConstraint() override;
};

class StretchConstraint : public Constraint
{
private:
    Particle* p1, * p2;
    float threshold;

public:
    StretchConstraint(Particle*, Particle*, float);
    ~StretchConstraint();
    void projectConstraint() override;
};

class BendingConstraint : public Constraint
{
private:
    Particle* p1, * p2, * p3, * p4;
    float threshold;
public:
    BendingConstraint(Particle*, Particle*, Particle*, Particle*, float);
    ~BendingConstraint();
    void projectConstraint() override;
};