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
	Viewer(name)
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
	float deltaT = 1.0 / ImGui::GetIO().Framerate;
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
	//mParticles.update(deltaT);
	drawParticles(projView);
	glDisable(GL_DEPTH_TEST);
}


void ParticleViewer::drawParticles(const glm::mat4& projView)
{
	mModelShader->use();
	mModelShader->setMat4("uProjView", projView);
	mModelShader->setVec3("uLightPos", mLightPos);
	for (int i = 0; i < nexusParticles.size(); ++i)
	{
		glm::vec3 pos = nexusParticles[i].x;
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::scale(model, glm::vec3(5, 5, 5));
		model = glm::translate(model, pos);
			
		mModelShader->setMat4("uModel", model);
		mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
		mModelShader->setVec3("color", glm::vec3(1,0,0));
		mParticleModelSphere->drawObj();
		
	}
}