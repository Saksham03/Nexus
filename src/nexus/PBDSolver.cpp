#include "PBDSolver.h"

PBDSolver::PBDSolver()
	:PBDSolver(GRAVITY, std::vector<uPtr<NexusObject>>())
{}

PBDSolver::PBDSolver(vec3 gravity, std::vector<uPtr<NexusObject>> objects)
	: gravity(gravity), objects(std::move(objects))
{}


PBDSolver::~PBDSolver()
{}

void PBDSolver::update(float deltaTime)
{
	float dt = deltaTime / NUM_SOLVER_SUBSTEPS;
	for (int i = 0; i < NUM_SOLVER_SUBSTEPS; i++)
	{
		for (auto& obj : objects)
		{
			for (auto& particle : obj->getParticles())
			{
				if (particle->invMass > 0.0f)
				{
					// Handle gravity
					particle->v = particle->v + dt * gravity * particle->mass;
					particle->prevX = particle->x;
					particle->x += dt * particle->v;
				}
			}
		}

		for (auto& obj : objects)
		{
			obj->update(dt);	// solve constraints
		}

		for (auto& obj : objects)
		{
			for (auto& particle : obj->getParticles())
			{
				if (particle->invMass > 0.0)
				{
					particle->v = (particle->x - particle->prevX) / dt;
				}
			}
		}
	}
}

void PBDSolver::addObject(uPtr<NexusObject> p)
{
	objects.push_back(std::move(p));
}

void PBDSolver::precomputeConstraints()
{
	for (auto& obj : objects)
	{
		obj->preComputeConstraints();
	}
}


const std::vector<uPtr<NexusObject>>& PBDSolver::getObjects() const
{
	return objects;
}