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
	for (auto& obj : objects)
	{
		for (auto& particle : obj->particles)
		{
			if (particle->invMass > 0.0f)
			{
				// Handle gravity
				particle->v = particle->v + deltaTime * gravity;
				particle->prevX = particle->x;
				particle->x += deltaTime * particle->v;
			}
		}

		float dt = deltaTime / NUM_SOLVER_SUBSTEPS;
		for (int i = 0; i < NUM_SOLVER_SUBSTEPS; i++)
		{
			for (auto& c : obj->constraints)
			{
				c->projectConstraint();	// solve constraints
			}
		}

		for (auto& particle : obj->particles)
		{
			if (particle->invMass > 0.0)
			{
				particle->v = (particle->x - particle->prevX) / deltaTime;
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