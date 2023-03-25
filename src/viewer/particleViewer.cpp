#include "particleViewer.h"
#include <iostream>


int LENGTH = 10;
int BREADTH = 10;

std::vector<Particle> getTestSceneRowOfParticles() {
	
std::vector<Particle> particles;
	

	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH; j++) {
			particles.push_back(Particle(2.f));
			particles.back().x += glm::vec3(j * 3, 20, i * 3);
		}
	}
	return particles;
}

	/*if (j != 0) {
		std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(&particles[i * LENGTH + j - 1], &particles[i * LENGTH + j], 2.f);
		std::cout << (i * LENGTH + j - 1) << ", " << (i * LENGTH + j) << std::endl;
		constraints.push_back(std::move(d));
	}
	if (i != 0) {
		std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(&particles[(i - 1) * LENGTH + j], &particles[i * LENGTH + j], 2.f);
		std::cout << ((i - 1) * LENGTH + j) << ", " << (i * LENGTH + j) << std::endl;
		constraints.push_back(std::move(d));
	}
	std::unique_ptr<DistanceConstraint> d1 = std::make_unique<DistanceConstraint>(&particles[0], particles[0].x, 0.f);
	constraints.push_back(std::move(d1));
	std::unique_ptr<DistanceConstraint> d2 = std::make_unique<DistanceConstraint>(&particles[LENGTH - 1], particles[LENGTH - 1].x, 0.f);
	constraints.push_back(std::move(d2));*/


ParticleViewer::ParticleViewer(const std::string& name) :
	Viewer(name),
	solver(mkU<PBDSolver>())
{
	clearColor = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
	nexusParticles = getTestSceneRowOfParticles();
	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH - 1; j++) {
			std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(&nexusParticles[i * LENGTH + j], &nexusParticles[i * LENGTH + j + 1], 3.f);
			constraints.push_back(std::move(d));
		}
	}
	for (int i = 0; i < BREADTH - 1; i++) {
		for (int j = 0; j < LENGTH; j++) {
			std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(&nexusParticles[i * LENGTH + j], &nexusParticles[(i+1) * LENGTH + j], 3.f);
			constraints.push_back(std::move(d));
		}
	}
	std::unique_ptr<DistanceConstraint> d1 = std::make_unique<DistanceConstraint>(&nexusParticles[0], nexusParticles[0].x, 0.f);
	constraints.push_back(std::move(d1));
	std::unique_ptr<DistanceConstraint> d2 = std::make_unique<DistanceConstraint>(&nexusParticles[LENGTH - 1], nexusParticles[LENGTH - 1].x, 0.f);
	constraints.push_back(std::move(d2));
	/*std::unique_ptr<DistanceConstraint> d = std::make_unique<DistanceConstraint>(&nexusParticles[0], nexusParticles[0].x, 0.f);
	constraints.push_back(std::move(d));
	for (int i = 0; i < nexusParticles.size() - 1; i++) {
		std::unique_ptr<StretchConstraint> d = std::make_unique<StretchConstraint>(&nexusParticles[i], &nexusParticles[i+1], 2.f);
		constraints.push_back(std::move(d));
	}*/
	
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
	//addRope();
	addCloth();
	solver->precomputeConstraints();
}

void ParticleViewer::addRope()
{
	uPtr<NexusCloth> cloth = mkU<NexusCloth>(1.0f, 3.0f);
	float dist = 3.0f;
	for (int i = 0; i < 20; i++)
	{
		float mass = 1.0f;
		if (i == 0)
		{
			mass = INFINITY;
		}
		uPtr<Particle> p = mkU<Particle>(vec3(i * -dist, 10, 0),
			vec3(0.0f),
			mass,
			0);
		cloth->addParticle(std::move(p));
	}
	cloth->setRowsAndColumns(20, 1);
	solver->addObject(std::move(cloth));
}

void ParticleViewer::addCloth()
{
	uPtr<NexusCloth> cloth = mkU<NexusCloth>(0.5f, 3.0f);
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
			uPtr<Particle> p = mkU<Particle>(vec3(i * -dist, 10, j * dist),
				vec3(0.0f),
				//0.5f * (std::abs(i-2.0f)+std::abs(j-2.0f)) + 1.0f,
				mass,
				0);
			cloth->addParticle(std::move(p));
		}
	}
	cloth->setRowsAndColumns(10, 10);
	solver->addObject(std::move(cloth));
}

void ParticleViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	float deltaT = ImGui::GetIO().DeltaTime;
	glm::mat4 projView = mCamera.getProjView();
	int subStepSize = 10;
	float substep = deltaT / subStepSize;
	
	for (auto& p : nexusParticles) {
		//euler
		p.v += glm::vec3(0, -14.8, 0) * deltaT;
		p.prevx = p.x;
		p.x += p.v * deltaT;
	}
	

	for (int i = 0; i < subStepSize; i++) {
			for (auto& c : constraints) {
				c->projectConstraint();
			}		
	}
	
	for (auto& p : nexusParticles) {
	//update velocity
		p.v = (p.x - p.prevx) / deltaT;
	}

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