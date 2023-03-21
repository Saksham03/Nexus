#pragma once

#include "Particle.h"
#include "Constraint.h"
#include "NexusObject.h"
#include <vector>

class NexusCloth : public NexusObject {
public:
	NexusCloth();
	~NexusCloth();
		/// <summary>
	/// clloth
	/// </summary>
	/// <param name="type">Type of Nexus Object</param>
	/// <param name=""></param>
	NexusCloth(NEXUS_OBJECT_TYPE, std::vector<Particle>);
	void solve() override;
};