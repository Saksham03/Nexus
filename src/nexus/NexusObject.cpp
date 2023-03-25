#include "NexusObject.h"
#include <stdio.h>

NexusObject::NexusObject(NEXUS_OBJECT_TYPE type, std::vector<Particle> particles, glm::vec3 f)
	: type(type), particles(particles), force(f)
{}

NexusObject::NexusObject()
	: type(NEXUS_OBJECT_TYPE::UNDEFINED), particles(), force()
{}

NexusObject::~NexusObject()
{}