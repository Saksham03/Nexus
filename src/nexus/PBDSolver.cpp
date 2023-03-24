#include "PBDSolver.h"

PBDSolver::PBDSolver()
	:PBDSolver(GRAVITY, std::vector<uPtr<Particle>>())
{}

PBDSolver::PBDSolver(vec3 gravity, std::vector<uPtr<Particle>> particles)
	: gravity(gravity), particles(std::move(particles))
{}


PBDSolver::~PBDSolver()
{}

void PBDSolver::update(float deltaTime)
{
	for (auto& particle : particles)
	{
		// Handle gravity
		particle->v = particle->v + deltaTime * gravity;
		particle->x += deltaTime * particle->v;
	}
}

void PBDSolver::addParticle(uPtr<Particle> p)
{
	particles.push_back(std::move(p));
}

const std::vector<uPtr<Particle>>& PBDSolver::getParticles() const
{
	return particles;
}
