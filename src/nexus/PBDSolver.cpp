#include "PBDSolver.h"

PBDSolver::PBDSolver()
	:PBDSolver(GRAVITY, std::vector<uPtr<NexusObject>>())
{}

PBDSolver::PBDSolver(vec3 gravity, std::vector<uPtr<NexusObject>> objects)
	: gravity(gravity), objects(std::move(objects)), collConstraints(std::vector<uPtr<Constraint>>())
{}


PBDSolver::~PBDSolver()
{}

void PBDSolver::update(float deltaTime)
{
	generateCollisions();

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
			for (auto& c : collConstraints)
			{
				c->projectConstraint();
			}

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

void PBDSolver::generateCollisions()
{
	collConstraints.clear();

	for (int objI = 0; objI < objects.size(); objI++)
	{
		NexusObject* obj1 = objects[objI].get();

		for (int pI = 0; pI < obj1->particles.size(); pI++)
		{
			Particle* p1 = obj1->particles[pI].get();

			for (int objJ = 0; objJ < objects.size(); objJ++)
			{
				if (objI == objJ)
				{
					continue;
				}

				NexusObject* obj2 = objects[objJ].get();
				for (int pJ = 0; pJ < obj2->particles.size(); pJ++)
				{
					Particle* p2= obj2->particles[pJ].get();

					collConstraints.push_back(mkU<ParticleParticleCollisionConstraint>(p1, p2));
				}
			}
		}
	}
	
	return;

	for (auto& obj1 : objects)
	{
		for (auto& p1 : obj1->particles)
		{
			for (auto& obj2 : objects)
			{
				if (obj1 == obj2)
				{
					return;
				}

				for (auto& p2 : obj2->particles)
				{
					if (p1 == p2)
					{
						continue;
					}

					collConstraints.push_back(mkU<ParticleParticleCollisionConstraint>(p1.get(), p2.get()));
				}
			}
		}
	}
}

const std::vector<uPtr<NexusObject>>& PBDSolver::getObjects() const
{
	return objects;
}