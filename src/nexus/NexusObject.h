#pragma once

#include "Particle.h"
#include "Constraint.h"
#include <vector>

enum class NEXUS_OBJECT_TYPE { UNDEFINED = -1, CLOTH = 0, RIGIDBODY = 100 };

class NexusObject {
	friend class PBDSolver;
private:
	static int lastId;

protected:
	std::vector<uPtr<Particle>> particles;
	std::vector<uPtr<Constraint>> constraints;

public:
	NEXUS_OBJECT_TYPE type;

	NexusObject(NEXUS_OBJECT_TYPE, std::vector<uPtr<Particle>>);
	~NexusObject();

	void addParticle(uPtr<Particle> p);
	void addStretchConstraint(Particle* p1, Particle* p2, double distance, double stiffness);
	void addBendingConstraint(Particle* p1, Particle* p2, Particle* p3, Particle* p4, double stiffness);
	void fixParticle(Particle* p);
	const std::vector<uPtr<Particle>>& getParticles() const;
	virtual void preComputeConstraints() = 0;
	static int getObjectID();
};