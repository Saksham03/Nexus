#include "particleViewer.h"
#include <iostream>
#include <chrono>

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
	addCloth(0);
	//addCloth(1);
	addBall();
	addCube(1);
	addCube(0);
	addCube(2);
	//addCube(3);
	//addCube(4);

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
			uPtr<Particle> p = mkU<Particle>(glm::vec3(100.0f, 150.0f, j * 5.0f), 
											vec3(0.0f),
											-1,
											mass,
											FIXED_PARTICLE_SIZE);

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
	p->x = glm::vec3(80.0f, 50.0f, 80.0f);
	p->prevX = p->x;
	p->radius = 40.0f;
	p->color = vec3(1, 0, 0);
	p->phase = -1;
	solver->setBigBoi(std::move(p));
	/*ball->addParticle(std::move(p));
	ball->setLengthAndBreadth(1, 1);*/
	//solver->addObject(std::move(ball));
}

void ParticleViewer::addCloth(int idx)
{
	int LENGTH = 50;
	int BREADTH = 50;

	uPtr<NexusCloth> cloth = mkU<NexusCloth>();

	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH; j++) {
			float mass = 2.0f;
			if (/*idx == 0 && */(i == 0 || i == BREADTH - 1) && (j == 0 || j == LENGTH - 1))
			{
				mass = -1.0f;
			}
			uPtr<Particle> p = mkU<Particle>(vec3(j * 5.0f, 100 + 150 * idx, i * 5.0f),
											vec3(0.0f),
											-1,
											mass,
											FIXED_PARTICLE_SIZE,
											vec3(0, idx, 1));

			cloth->addParticle(std::move(p));
		}
	}

	cloth->setLengthAndBreadth(LENGTH, BREADTH);
	solver->addObject(std::move(cloth));
}

void ParticleViewer::addCube(int off)
{
	int LENGTH = 5;
	int BREADTH = 5;
	int HEIGHT = 5;

	int phase = NexusObject::getObjectID();

	uPtr<NexusRigidBody> cube = mkU<NexusRigidBody>();
	vec3 offset(50 + off * 50, 200 + off*100, 50 );
	mat3 rot = mat3(-0.9036922, 0.0000000, -0.4281827,
	-0.3909073, 0.4080821, 0.8250215,
	0.1747337, 0.9129453, -0.3687806);

	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH; j++) {
			for (int k = 0; k < HEIGHT; k++) {
				float mass = 10.0f;

				uPtr<Particle> p = mkU<Particle>(vec3(i * 5.0f, k * 5.0f, j * 5.0f) + offset,
												vec3(0.0f), 
												phase,
												mass,
												FIXED_PARTICLE_SIZE,
												vec3(1, 1, 1));

				cube->addParticle(std::move(p));
			}
		}
	}

	//cube->setLengthAndBreadth(LENGTH, BREADTH);
	solver->addObject(std::move(cube));
}

void ParticleViewer::drawScene()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	float deltaT = ImGui::GetIO().DeltaTime;
	glm::mat4 projView = mCamera.getProjView();

	drawGridGround(projView);
	auto begin = std::chrono::steady_clock::now();
	solver->update(FIXED_TIMESTEP);
	auto end = std::chrono::steady_clock::now();
	std::cout << "Time difference = " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1000.0f << "ms" << std::endl;
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