#include "particleViewer.h"

ParticleViewer::ParticleViewer(const std::string& name) :
	Viewer(name),
	solver(mkU<PBDSolver>())
{
	clearColor = ImVec4(0.6f, 0.6f, 0.6f, 1.00f);
	mParticleModel = std::make_unique<ObjModel>();
	mRocketModel = std::make_unique<ObjModel>();
	mParticleModelSphere = std::make_unique<ObjModel>();
	solver->addParticle(std::move(mkU<Particle>(vec3(0, 10, 0), vec3(0.0f), 5.0f, 0)));
	mParticleModel->loadObj("../obj/cube.obj");
	mRocketModel->loadObj("../obj/cone.obj");
	mParticleModelSphere->loadObj("../obj/sphere.obj");
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

ParticleViewer::~ParticleViewer()
{
}

void ParticleViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	float deltaT = ImGui::GetIO().DeltaTime;
	glm::mat4 projView = mCamera.getProjView();
	drawGridGround(projView);
	solver->update(deltaT);
	drawParticles(projView);
	glDisable(GL_DEPTH_TEST);
}


void ParticleViewer::drawParticles(const glm::mat4& projView)
{
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);

	glm::vec3 color = vec3(1, 0, 0);

	for (auto& particle : solver->getParticles())
	{
		glm::vec3 pos = particle->x;
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::scale(model, glm::vec3(5, 5, 5));
		model = glm::translate(model, pos);

		mModelShader->setMat4("uModel", model);
		mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
		mModelShader->setVec3("color", glm::vec3(1, 0, 0));
		//mModelShader->setFloat("uAlpha", alpha);
		mParticleModelSphere->drawObj();
	}
}