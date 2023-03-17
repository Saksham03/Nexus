#include "NexusObject.h"
#include <stdio.h>

NexusObject::NexusObject()
	: type(NEXUS_OBJECT_TYPE::UNDEFINED)
{}

NexusObject::NexusObject(NEXUS_OBJECT_TYPE type, std::vector<Particle> particles)
	: type(type), particles(particles)
{}

NexusObject::~NexusObject()
{}