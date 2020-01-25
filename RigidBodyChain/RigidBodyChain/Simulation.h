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
	Matrix ground;
	vector<Matrix> cubes;

	//btDefaultCollisionConfiguration* collisionConfiguration;
	//btCollisionDispatcher* dispatcher;
	//btBroadphaseInterface* overlappingPairCache;
	//btSequentialImpulseConstraintSolver* solver;
	//btAlignedObjectArray<btCollisionShape*> collisionShapes;
	//vector< btRigidBody*> bodies;
	//btDiscreteDynamicsWorld* dynamicsWorld;


	mt19937 gen{ std::random_device{}() };

	void Init();
	void Reset();
	void Update(float dt);
	void Update();

	void TestInit();



	btMultiBody* createFeatherstoneMultiBody_testMultiDof(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical = false, bool floating = false);
	void addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
	void addBoxes_testMultiDof();

	const static bool g_floatingBase = false;
	//static bool g_firstInit = true;
	//static float scaling = 0.4f;
	const float friction = 1.;
	//static int g_constraintSolverType = 0;

#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

	//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024)

#define START_POS_X -5
//#define START_POS_Y 12
#define START_POS_Y 2
#define START_POS_Z -3

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btCollisionDispatcher* m_dispatcher;
	btDbvtBroadphase* m_broadphase;
	btMultiBodyConstraintSolver* m_solver;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
	
};
