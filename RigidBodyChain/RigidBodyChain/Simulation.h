#pragma once
#include <d3d11.h>
#include <vector>
#include <math.h> 
#include <DirectXMath.h>
#include <SimpleMath.h>
#include "Graphics/Vertex.h"
#include <random>
#include "btBulletDynamicsCommon.h"
#include <stdio.h>

using namespace std;
using namespace DirectX;
using namespace DirectX::SimpleMath;

class Simulation
{
public:

	float delta_time;
	float time;
	bool paused;

	~Simulation();

	float simulationSpeed;
	Matrix ground;
	vector<Matrix> cubes;
	
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btBroadphaseInterface* overlappingPairCache;
	btSequentialImpulseConstraintSolver* solver;
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	btDiscreteDynamicsWorld* dynamicsWorld;


	mt19937 gen{ std::random_device{}() };

	void Init();
	void Reset();
	void Update(float dt);
	void Update();

	void TestInit();
};

