#include "particleViewer.h"
#include <iostream>


ParticleViewer::ParticleViewer(const std::string& name) :
	Viewer(name),
	solver(mkU<PBDSolver>())
{
	clearColor = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
	mParticleModelSphere = std::make_unique<ObjModel>();
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
	addBall();
	solver->precomputeConstraints();
}

void ParticleViewer::addRope()
{
	int LENGTH = 30;
	int BREADTH = 1;

	uPtr<NexusCloth> rope = mkU<NexusCloth>();

	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH; j++) {
			float mass = 2.0f;
			if (i == 0 && j == 0)
			{
				mass = -1.0f;
			}
			uPtr<Particle> p = mkU<Particle>(mass);
			p->x = glm::vec3(100.0f, 150.0f, j*5.0f);
			p->radius = 2.5f;
			rope->addParticle(std::move(p));
		}
	}

	rope->setLengthAndBreadth(LENGTH, BREADTH);
	solver->addObject(std::move(rope));
}

void ParticleViewer::addBall()
{
	uPtr<NexusCloth> ball = mkU<NexusCloth>();
	uPtr<Particle> p = mkU<Particle>(-1.0f);
	p->x = glm::vec3(100.0f, 50.0f, 100.0f);
	p->radius = 20.0f;
	p->color = vec3(1, 0, 0);
	solver->setBigBoi(std::move(p));
	/*ball->addParticle(std::move(p));
	ball->setLengthAndBreadth(1, 1);*/
	//solver->addObject(std::move(ball));
}

void ParticleViewer::addCloth()
{
	int LENGTH = 45;
	int BREADTH = 30;

	uPtr<NexusCloth> cloth = mkU<NexusCloth>();

	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH; j++) {
			float mass = 2.0f;
			if (i == 0 && (j == 0 || j == LENGTH - 1))
			{
				mass = -1.0f;
			}
			uPtr<Particle> p = mkU<Particle>(mass);
			p->radius = 2.5f;
			p->x = glm::vec3(j * 5.0f, 100, i * 5.0f);
			p->color = vec3(0, 0, 1);
			cloth->addParticle(std::move(p));
		}
	}

	cloth->setLengthAndBreadth(LENGTH, BREADTH);
	solver->addObject(std::move(cloth));
}

void ParticleViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	float deltaT = ImGui::GetIO().DeltaTime;
	glm::mat4 projView = mCamera.getProjView();

	drawGridGround(projView);
	solver->update(FIXED_TIMESTEP);
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
			vec3 scale = glm::vec3(particle->radius);
			model = glm::scale(model, scale);
			model = glm::translate(model, pos/scale);

			mModelShader->setMat4("uModel", model);
			mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
			mModelShader->setVec3("color", particle->color);
			//mModelShader->setFloat("uAlpha", alpha);
			mParticleModelSphere->drawObj();
		}
	}
}