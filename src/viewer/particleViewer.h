#pragma once
#include "viewer.h"
#include "aParticleSystem.h"
#include "objmodel.h"
#include "Particle.h"

class ParticleViewer : public Viewer
{
public:
	ParticleViewer(const std::string& name);
	virtual ~ParticleViewer();

	virtual void drawScene()override;
//	virtual void createGUIWindow() override;
//	virtual void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) override;

private:
	int mDemo = 0;	// 0 for particles, 1 for fireworks

	int mParticleModelType = 0; // 0 for cube, 1 for sphere

	std::vector<uPtr<Particle>> mParticles;

	std::unique_ptr<ObjModel> mParticleModel;
	std::unique_ptr<ObjModel> mParticleModelSphere;
	std::unique_ptr<ObjModel> mRocketModel;

	void drawParticles(const glm::mat4& projView);
};