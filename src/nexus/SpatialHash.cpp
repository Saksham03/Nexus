#include "SpatialHash.h"
#include <unordered_set>

SpatialHash::SpatialHash()
	: SpatialHash(SPATIAL_HASH_GRID_SIZE)
{}

SpatialHash::SpatialHash(float gridSize)
	: gridSize(gridSize)
{}

SpatialHash::~SpatialHash()
{}

bool SpatialHash::hasGridAt(const vec3 pos) const
{
	auto it = particleMap.find(toKey(pos));
	return it != particleMap.end();
}

bool SpatialHash::hasGridAt(const int64_t key) const
{
	auto it = particleMap.find(key);
	return it != particleMap.end();
}

int64_t SpatialHash::toKey(const vec3 pos) const
{
	int x = static_cast<int>(std::floor(pos.x / gridSize));
	int y = static_cast<int>(std::floor(pos.y / gridSize));
	int z = static_cast<int>(std::floor(pos.z / gridSize));

	// this hash is from Muller, from https://carmencincotti.com/2022-10-31/spatial-hash-maps-part-one/#:~:text=Spatial%20hash%20maps%20deal%20with,video%20game%20physics%20engines%2C%20etc.
	const int64_t hash = (x * 92837111) ^ (y * 689287499) ^ (z * 283923481);
	return std::abs(hash);
}

void SpatialHash::insertParticle(Particle* p)
{
	int64_t key = toKey(p->x);
	if (!hasGridAt(p->x))
	{
		particleMap[key] = std::vector<Particle*>();
	}
	particleMap[key].push_back(p);

	//std::unordered_set<int64_t> hashes;

	//for (int idx = 0; idx < 3; idx++)
	//{
	//	// insert into all 6 cardinal directions
	//	for (int i = -1; i < 2; i+=2)
	//	{
	//		vec3 pos(0.0f);
	//		pos[idx] += i * p->radius;

	//		key = toKey(p->x + pos);
	//		if (hashes.find(key) == hashes.end())
	//		{
	//			if (!hasGridAt(key))
	//			{
	//				particleMap[key] = std::vector<Particle*>();
	//			}
	//			particleMap[key].push_back(p);
	//		}
	//	}
	//}
}


void SpatialHash::clear()
{
	particleMap.clear();
}

const float SpatialHash::getGridSize() const
{
	return gridSize;
}

const std::vector<Particle*>& SpatialHash::getParticlesAt(int64_t key) const
{
	return particleMap.at(key);
}
