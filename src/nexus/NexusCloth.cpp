#include "NexusCloth.h"

NexusCloth::NexusCloth()
	: NexusCloth(std::vector<uPtr<Particle>>(), 0.8f, 0.5f)
{}

NexusCloth::NexusCloth(float stiffness, float restLength)
	: NexusCloth(std::vector<uPtr<Particle>>(), stiffness, restLength)
{}

NexusCloth::NexusCloth(std::vector<uPtr<Particle>> particles, float stiffness, float restLength)
	: NexusObject(NEXUS_OBJECT_TYPE::CLOTH, std::move(particles)), stiffness(stiffness), restLength(restLength),
	stretchConstraints(std::unordered_set<std::pair<Particle*, Particle*>, PairHash>())
{}

NexusCloth::~NexusCloth()
{}

void NexusCloth::setRowsAndColumns(int rows, int columns)
{
	nRows = rows;
	nColumns = columns;
}

void NexusCloth::update(float deltaTime)
{
	DistanceConstraint c(stiffness, restLength);

	for (auto iter = stretchConstraints.begin(); iter != stretchConstraints.end(); iter++)
	{
		c.projectConstraint(new Particle * [] {iter->first, iter->second}, deltaTime);
	}
}

void NexusCloth::preComputeConstraints()
{
	for (int i = 0; i < nRows; i++)
	{
		for (int j = 0; j <nColumns; j++)
		{
			int idx = i * nColumns + j;

			std::pair<Particle*, Particle*> key;

			if (i > 0)
			{
				int adjIdx = (i - 1) * nColumns + j;
				key = std::make_pair(particles[idx].get(), particles[adjIdx].get());
				if (stretchConstraints.find(key) == stretchConstraints.end())
				{
					stretchConstraints.insert(key);
				}
			}
			if (i < nRows - 1)
			{
				int adjIdx = (i + 1) * nColumns + j;
				key = std::make_pair(particles[idx].get(), particles[adjIdx].get());
				if (stretchConstraints.find(key) == stretchConstraints.end())
				{
					stretchConstraints.insert(key);
				}
			}
			if (j > 0)
			{
				int adjIdx = i * nColumns + (j - 1);
				key = std::make_pair(particles[idx].get(), particles[adjIdx].get());
				if (stretchConstraints.find(key) == stretchConstraints.end())
				{
					stretchConstraints.insert(key);
				}
			}
			if (j < nColumns - 1)
			{
				int adjIdx = i * nColumns + (j + 1);
				key = std::make_pair(particles[idx].get(), particles[adjIdx].get());
				if (stretchConstraints.find(key) == stretchConstraints.end())
				{
					stretchConstraints.insert(key);
				}
			}
		}
	}
}
