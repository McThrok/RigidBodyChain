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

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"

#include "BulletCollision/CollisionShapes/btShapeHull.h"

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
	Matrix groundMatrix;
	vector<Matrix> cubes;
	vector<Matrix> cubes2;

	void Update(float dt);
	void Update();

	void Init();
	void InitBullet();
	void InitWorld();
	void InitTerrain();

	void InitMultiBody();
	btMultiBody* InitEmptyMultiBody(int numLinks);
	void InitMultiBodyLinks(btMultiBody* mb);
	void InitMultiBodyLinksColliders(btMultiBody* mb);

	void InitHandlers();
	btMultiBodyPoint2Point* AddHandle(int idx, btVector3 pivot);
	void UpdateMatrices();

	const float friction = 1.0f;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionDispatcher* m_dispatcher;
	btDbvtBroadphase* m_broadphase;
	btMultiBodyConstraintSolver* m_solver;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	btMultiBodyPoint2Point* handle1;
	btMultiBodyPoint2Point* handle2;
	const btVector3 halfSize = { 0.1, 0.5, 0.1 };


	btMultiBodyPoint2Point* handle3;
	btMultiBodyPoint2Point* handle4;

	void InitMultiBody2();
	void InitMultiBodyLinks2(btMultiBody* mb);

	void InitHandlers2();
	btMultiBodyPoint2Point* AddHandle2(int idx, btVector3 pivot);
	void UpdateMatrices2();
};
