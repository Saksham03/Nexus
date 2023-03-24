#pragma once

#include "Particle.h"
#include "Constraint.h"

class PBDSolver {
private:
	vec3 gravity;
	std::vector<uPtr<Particle>> particles;
public:
	PBDSolver();
	PBDSolver(vec3 gravity, std::vector<uPtr<Particle>> particles);
	~PBDSolver();
	void update(float deltaTime);

	void addParticle(uPtr<Particle> p);
	const std::vector<uPtr<Particle>>& getParticles() const;
};