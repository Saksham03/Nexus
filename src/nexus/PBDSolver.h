#pragma once

#include "Particle.h"
#include "Constraint.h"

class PBDSolver {
public:
	PBDSolver();
	~PBDSolver();
	void solve();
};