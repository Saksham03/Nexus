#pragma once
#include "viewer.h"
#include "aParticleSystem.h"
#include "objmodel.h"
#include "PBDSolver.h"
#include "NexusCloth.h"
#include "Particle.h"
#include "Constraint.h"

class ParticleViewer : public Viewer
{
public:
	ParticleViewer(const std::string& name);
	virtual ~ParticleViewer();

	virtual void drawScene()override;
	void setupScene();
	void addRope();
	void addCloth();
//	virtual void createGUIWindow() override;
//	virtual void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) override;

private:
	int mDemo = 0;	// 0 for particles, 1 for fireworks

	int mParticleModelType = 0; // 0 for cube, 1 for sphere

	uPtr<PBDSolver> solver;
	AParticleSystem mParticles;
	std::vector<Particle> nexusParticles;
	std::vector<std::unique_ptr<Constraint>> constraints;

	std::unique_ptr<ObjModel> mParticleModel;
	std::unique_ptr<ObjModel> mParticleModelSphere;
	std::unique_ptr<ObjModel> mRocketModel;

	void drawParticles(const glm::mat4& projView);
};