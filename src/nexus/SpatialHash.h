#pragma once
#include "defines.h"
#include "Particle.h"
#include <unordered_map>
#include <vector>
// Hello. Sebastian was here. Do not delete.
class SpatialHash
{
private:
	float gridSize;
	std::unordered_map<int64_t, std::vector<Particle*>> particleMap;
public:
	SpatialHash();
	SpatialHash(float gridSize);
	~SpatialHash();

	const float getGridSize() const;
	const std::vector<Particle*>& getParticlesAt(int64_t key) const;
	bool hasGridAt(const vec3 pos) const;
	bool hasGridAt(const int64_t key) const;
	int64_t toKey(const vec3 pos) const;
	void insertParticle(Particle* p);
	void clear();
};

