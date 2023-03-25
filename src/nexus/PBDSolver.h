#pragma once

#include "Particle.h"
#include "Constraint.h"

class PBDSolver {
public:
	PBDSolver();
	~PBDSolver();
	void solve();
	void updateStateEuler(std::vector<Particle>&, std::vector<Particle>&, float);
};