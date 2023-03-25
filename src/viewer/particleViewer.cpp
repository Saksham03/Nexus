#include "particleViewer.h"

ParticleViewer::ParticleViewer(const std::string& name) :
	Viewer(name),
	solver(mkU<PBDSolver>())
{
	clearColor = ImVec4(0.6f, 0.6f, 0.6f, 1.00f);
	mParticleModel = std::make_unique<ObjModel>();
	mRocketModel = std::make_unique<ObjModel>();
	mParticleModelSphere = std::make_unique<ObjModel>();
	mParticleModel->loadObj("../obj/cube.obj");
	mRocketModel->loadObj("../obj/cone.obj");
	mParticleModelSphere->loadObj("../obj/sphere.obj");
	setupScene();
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

ParticleViewer::~ParticleViewer()
{
}

void ParticleViewer::setupScene()
{
	addRope();
	addCloth();
}

void ParticleViewer::addRope()
{
	uPtr<NexusCloth> cloth = mkU<NexusCloth>(0.9f, 3.0f);
	float dist = 3.0f;
	for (int i = 0; i < 15; i++)
	{
		//for (int j = 0; j < 5; j++)
		//{
		//	
		//}

		float mass = 1.0f;
		if (i == 0/* || i == 4) && (j == 0)*/)
		{
			mass = INFINITY;
		}
		uPtr<Particle> p = mkU<Particle>(vec3(i * -dist, 10, 0),
			vec3(0.0f),
			//0.5f * (std::abs(i-2.0f)+std::abs(j-2.0f)) + 1.0f,
			mass,
			0);
		cloth->addParticle(std::move(p));
	}
	solver->addObject(std::move(cloth));
}

void ParticleViewer::addCloth()
{
	uPtr<NexusCloth> cloth = mkU<NexusCloth>(0.9f, 3.0f);
	float dist = 3.0f;
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			float mass = 1.0f;
			if ((i == 0 || i == 9) && (j == 0))
			{
				mass = INFINITY;
			}
			uPtr<Particle> p = mkU<Particle>(vec3(i * -dist, 10, 0),
				vec3(0.0f),
				//0.5f * (std::abs(i-2.0f)+std::abs(j-2.0f)) + 1.0f,
				mass,
				0);
			cloth->addParticle(std::move(p));
		}
	}
	solver->addObject(std::move(cloth));
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

	for (auto& obj : solver->getObjects())
	{
		for (auto& particle : obj->getParticles())
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
}