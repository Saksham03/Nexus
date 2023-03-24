#include "Particle.h"

Particle::Particle(vec3 position, vec3 velocity, float mass, int phase)
	: x(position), v(velocity), invMass(1.0f/mass), phase(phase)
{}
