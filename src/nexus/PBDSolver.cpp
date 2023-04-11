#include "PBDSolver.h"

PBDSolver::PBDSolver()
	:PBDSolver(GRAVITY, std::vector<uPtr<NexusObject>>())
{}

PBDSolver::PBDSolver(vec3 gravity, std::vector<uPtr<NexusObject>> objects)
	: gravity(gravity), objects(std::move(objects)), collConstraints(std::vector<uPtr<Constraint>>()),
	particleHash(SpatialHash(SPATIAL_HASH_GRID_SIZE)),
	alreadyCheckedCollisions()
{}


PBDSolver::~PBDSolver()
{}

void PBDSolver::update(float deltaTime)
{
	generateSpatialHash();
	generateCollisions();

	//float dt = deltaTime / NUM_SOLVER_SUBSTEPS;

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

		for (int i = 0; i < NUM_SOLVER_ITERATIONS; i++)
		{
			for (auto& c : collConstraints)
			{
				c->projectConstraint();
			}

			for (auto& c : obj->constraints)
			{
				c->projectConstraint();	// solve constraints
			}

			for (auto& particle : obj->particles)
			{
				if (particle->invMass > 0.0f)
				{
					if (particle->x.y < 0.0f)
					{
						particle->x.y = 0.0f;;

						vec3 delta = particle->x - particle->prevX;
						vec3 proj = /*glm::dot(delta, vec3(0,1,0))*/ vec3(0, delta.y, 0) /** vec3(0, 1, 0)*/;
						vec3 tangential = (delta - proj);
						float tangentialDist = glm::length(tangential);

						float us = 0.05f;
						float uk = 0.002f;

						float staticFric = glm::length(delta) * us;
						delta = /*particle->invMass **/ tangential;
						if (tangentialDist >= staticFric + 0.0001f)
						{
							float dynamicFric = glm::length(delta) * uk;
							delta *= std::min((dynamicFric / tangentialDist), 1.0f);
						}

						particle->x -= delta;
					}
				}
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

void PBDSolver::generateSpatialHash()
{
	particleHash.clear();
	for (auto& obj : objects)
	{
		for (auto& p : obj->particles)
		{
			particleHash.insertParticle(p.get());
		}
	}
}

void PBDSolver::generateCollisions()
{
	collConstraints.clear();
	alreadyCheckedCollisions.clear();

	for (int objI = 0; objI < objects.size(); objI++)
	{
		NexusObject* obj1 = objects[objI].get();

		for (int pIdx = 0; pIdx < obj1->particles.size(); pIdx++)
		{
			Particle* p1 = obj1->particles[pIdx].get();
			collConstraints.push_back(mkU<ParticleParticleCollisionConstraint>(p1, pBigBoi.get()));

			for (int i = -1; i < 2; i++)
			{
				for (int j = -1; j < 2; j++)
				{
					for (int k = -1; k < 2; k++)
					{
						int64_t key = particleHash.toKey(p1->x + vec3(i * particleHash.getGridSize(),
																	j * particleHash.getGridSize(),
																	k * particleHash.getGridSize()));

						if (particleHash.hasGridAt(key))
						{
							const std::vector<Particle*>& particles = particleHash.getParticlesAt(key);
							for (auto p2 : particles)
							{
								if (p1 == p2)
								{
									continue;
								}
								if (ParticleParticleCollisionConstraint::areParticlesColliding(p1, p2)
									&& alreadyCheckedCollisions.find(std::pair<Particle*, Particle*>(p1, p2)) == alreadyCheckedCollisions.end())
								{
									// avoid checking collisions between the same particles twice
									collConstraints.push_back(mkU<ParticleParticleCollisionConstraint>(p1, p2));
								}
							}
						}
					}
				}
			}
			/*for (int objJ = 0; objJ < objects.size(); objJ++)
			{
				NexusObject* obj2 = objects[objJ].get();
				for (int pJ = 0; pJ < obj2->particles.size(); pJ++)
				{
					Particle* p2= obj2->particles[pJ].get();
					 if (p1 == p2)
					 {
						continue;
					 }
					 if (ParticleParticleCollisionConstraint::areParticlesColliding(p1, p2))
					 {
						 collConstraints.push_back(mkU<ParticleParticleCollisionConstraint>(p1, p2));
					 }
				}
			}*/
		}
	}
}

const std::vector<uPtr<NexusObject>>& PBDSolver::getObjects() const
{
	return objects;
}

void PBDSolver::setBigBoi(uPtr<Particle>&& p)
{
	this->pBigBoi = std::move(p);
}
