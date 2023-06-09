#include "Particle.h"

Particle::Particle(vec3 position, vec3 velocity, float mass, int phase)
	: x(position), v(velocity), mass(mass), invMass(mass == INFINITY ? 0.0f : 1.0f / mass), phase(phase)
{}
