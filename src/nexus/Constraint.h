#pragma once
#include "Particle.h"
#include <vector>
#include <Eigen\Dense>
using namespace Eigen;

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
    double stiffness;
    double function; //evaluation of the function at the current state

public:
    Constraint();
    Constraint(double, CONSTRAINT_TYPE);
    virtual ~Constraint();
    bool isConstraintSatisfied();
    virtual void projectConstraint(int iteration) = 0;
};

class DistanceConstraint : public Constraint
{
private:
    Particle* p;
    vec3 ref;
    double radius;

public:
    DistanceConstraint(Particle*, glm::vec3, double, double stiffness = 1.0f);
    ~DistanceConstraint();
    void projectConstraint(int iteration) override;
};

class StretchConstraint : public Constraint
{
private:
    Particle* p1, * p2;
    double threshold;

public:
    StretchConstraint(Particle*, Particle*, double, double stiffness = 1.0f);
    ~StretchConstraint();
    void projectConstraint(int iteration) override;
};

class ParticleParticleCollisionConstraint : public Constraint
{
private:
    Particle* p1, * p2;

public:
    ParticleParticleCollisionConstraint(Particle*, Particle*, double stiffness = 1.0f);
    ~ParticleParticleCollisionConstraint();
    void projectConstraint(int iteration) override;

    static bool areParticlesColliding(Particle*, Particle*);
};

class ShapeMatchingConstraint : public Constraint
{
private:
    std::vector<Particle*> particles;
    vec3 com_rest;                  	// center of mass at rest
    std::vector<vec3> q;                // rest config positions - rest center of mass positions
    Quaterniond prevRot;
    vec3 currCOM;
    mat3 shapeMatchingMat;
    void updateCurrentCom();
    void extractRotation(const Matrix3d& A, Quaterniond& q, const unsigned int maxIter) const;
public:
    ShapeMatchingConstraint(std::vector<Particle*> particles, double stiffness = 1.0f);
    ~ShapeMatchingConstraint();
    void projectConstraint(int iteration) override;
    const vec3& getCurrentCOM() const;
    const mat3& getShapeMatchingMatrix() const;
};

class BendingConstraint : public Constraint
{
private:
    Particle* p1, * p2, * p3, * p4;
    double threshold;
public:
    BendingConstraint(Particle* p1, Particle* p2, Particle* p3, Particle* p4, double threshold, double stiffness = 1.0f);
    ~BendingConstraint();
    void projectConstraint(int iteration) override;
};