#include "particleViewer.h"
#include <iostream>
#include <chrono>
#include "tiny_obj_loader.h"
#include "Helper.h"

#define VOXELIZER_IMPLEMENTATION
#include "voxelizer.h"

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
	//addCube(1);
	//addCube(0);
	//addCube(2);
	//addCube(3);
	//addCube(4);

	addMesh();

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
			if (idx == 0 && (i == 0 || i == BREADTH - 1) && (j == 0 || j == LENGTH - 1))
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
	int LENGTH = 9;
	int BREADTH = 9;
	int HEIGHT = 9;

	int phase = NexusObject::getObjectID();

	uPtr<NexusRigidBody> cube = mkU<NexusRigidBody>(1.0f);
	vec3 offset(50 + off * 50, 200 + off*100, 50 );
	mat3 rot = mat3(-0.9036922, 0.0000000, -0.4281827,
	-0.3909073, 0.4080821, 0.8250215,
	0.1747337, 0.9129453, -0.3687806);

	for (int i = 0; i < BREADTH; i++) {
		for (int j = 0; j < LENGTH; j++) {
			for (int k = 0; k < HEIGHT; k++) {
				if (i > 0 && i < BREADTH - 1 &&
					j > 0 && j < LENGTH - 1 &&
					k > 0 && k < HEIGHT - 1)
				{
					continue;
				}
				float mass = 2.0f;

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

vx_mesh_t* LoadFromFileAndVoxelize(const char* filename, float voxelsizex, float voxelsizey, float voxelsizez, float precision, std::vector<vec3> &verts)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	std::string warn;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename);
	if (!err.empty()) {
		printf("err: %s\n", err.c_str());
	}

	if (!ret) {
		printf("failed to load : %s\n", filename);
		return nullptr;
	}

	if (shapes.size() == 0) {
		printf("err: # of shapes are zero.\n");
		return nullptr;
	}

	// Only use first shape.
	std::vector<int> indices;
	for (tinyobj::index_t idxs : shapes[0].mesh.indices)
	{
		indices.push_back(idxs.vertex_index);
	}

	for (size_t v = 0; v < attrib.vertices.size() / 3; v++) {
		vec3 vert(attrib.vertices[3 * v + 0], attrib.vertices[3 * v + 1], attrib.vertices[3 * v + 2]);
		verts.push_back(vert);
	}
	return Helper::Voxelize(verts, indices, voxelsizex, voxelsizey, voxelsizez, precision);
}

void ParticleViewer::addMesh()
{
	mCustomMesh = mkU<ObjModel>();
	mCustomMesh->loadObj("../obj/cube.obj");
	
	vx_mesh_t* voxelPtr;
	std::vector<vec3> verts;
	voxelPtr = LoadFromFileAndVoxelize("../obj/cube.obj", FIXED_PARTICLE_SIZE * 0.1f, FIXED_PARTICLE_SIZE * 0.1f, FIXED_PARTICLE_SIZE * 0.1f, 0.01f, verts);

	int phase = NexusObject::getObjectID();

	uPtr<NexusRigidBody> rb = mkU<NexusRigidBody>(1.0f);
	vec3 offset = vec3(50.0f, 200.0f,50.0f);
	for (int i = 0; i < voxelPtr->nvertices; i++)
	{
		vx_vertex_t v = voxelPtr->vertices[i];
		vec3 pos = vec3(v.x, v.y, v.z);

		float mass = 2.0f;
		uPtr<Particle> p = mkU<Particle>(mat3(FIXED_PARTICLE_SIZE *6.0f) * pos + offset,
			vec3(0.0f),
			phase,
			mass,
			FIXED_PARTICLE_SIZE,
			vec3(1, 1, 1));

		rb->addParticle(std::move(p));
	}

	for (int i = 0; i < verts.size(); i++)
	{
		verts[i] = mat3(FIXED_PARTICLE_SIZE * 6.0f) * verts[i] + offset;
	}

	rb->setOriginalVerts(verts);
	solver->addObject(std::move(rb));
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
		NexusRigidBody* rb = dynamic_cast<NexusRigidBody*>(obj.get());
		if (false && rb != nullptr)
		{
			vec3 offset = vec3(50.0f, 200.0f, 50.0f);

			for (auto pos : rb->getMovedVertices())
			{
				vec3 p = mat3(FIXED_PARTICLE_SIZE * 6.0f) * pos + offset;
				glm::mat4 model = glm::mat4(1.0f);
				vec3 scale = glm::vec3(FIXED_PARTICLE_SIZE);
				model = glm::scale(model, scale);
				model = glm::translate(model, pos / scale);

				mModelShader->setMat4("uModel", model);
				//mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
				mModelShader->setVec3("color", vec3(1,1,0));
				//mModelShader->setFloat("uAlpha", 0.5f);
				mParticleModelSphere->drawObj();
			}
		}
		else
		{
			for (auto& particle : obj->getParticles())
			{
				glm::vec3 pos = particle->x;
				glm::mat4 model = glm::mat4(1.0f);
				vec3 scale = glm::vec3(particle->radius);
				model = glm::scale(model, scale);
				model = glm::translate(model, pos / scale);

				mModelShader->setMat4("uModel", model);
				//mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
				mModelShader->setVec3("color", particle->color);
				//mModelShader->setFloat("uAlpha", 0.5f);
				mParticleModelSphere->drawObj();
			}
		}
	}

	//vec3 pos = vec3(0.0f);
	//mat4 model = mat4(1.0);

	//float size = 70.0f;
	//model = glm::rotate(model, 110.0f, vec3(0.0f, 1.0f, 0.0f));
	//model = glm::scale(model, vec3(size, size, size));
	//model = glm::translate(model, vec3(0, 300.0f, 0.0f)/vec3(size));
	//mModelShader->setMat4("uModel", model);
	//mModelShader->setMat3("uModelInvTr", glm::mat3(glm::transpose(glm::inverse(model))));
	//mModelShader->setVec3("color", vec3(0.0, 1.0, 1.0));
	//mModelShader->setFloat("uAlpha", 1.0f);

	//mCustomMesh->drawObj();
}