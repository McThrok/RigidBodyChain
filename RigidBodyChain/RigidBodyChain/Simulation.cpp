#include "Simulation.h"

void Simulation::Init()
{
	time = 0;
	simulationSpeed = 1;
	paused = true;
	delta_time = 1.0 / 60;

	Reset();
	TestInit();

	auto multiBody = m_dynamicsWorld->getMultiBody(0);
	cubes.resize(multiBody->getNumLinks());

	UpdateMatrices();
}

void Simulation::Reset()
{
}

void Simulation::UpdateMatrices()
{
	auto multiBody = m_dynamicsWorld->getMultiBody(0);

	for (int i = 0; i < multiBody->getNumLinks(); i++)
	{
		auto transform = multiBody->getLink(i).m_collider->getWorldTransform();
		auto r = transform.getRotation();
		auto p = transform.getOrigin();
		cubes[i] = Matrix::CreateTranslation(-0.5 * Vector3::One) * Matrix::CreateScale(2 * Vector3(0.05, 0.37, 0.1))
			* Matrix::CreateFromQuaternion({ (float)r.getX(),(float)r.getY(),(float)r.getZ(),(float)r.getW() })
			* Matrix::CreateTranslation({ (float)p.getX(),(float)p.getY(),(float)p.getZ() });
	}
}

void Simulation::Update(float dt)
{
	//return;
	if (paused)
		return;

	time += dt / 1000;
	float timePerStep = delta_time / simulationSpeed;

	while (time >= timePerStep)
	{
		Update();
		time -= timePerStep;
	}
}

void Simulation::Update()
{
	m_dynamicsWorld->stepSimulation(delta_time);
	UpdateMatrices();
}

void Simulation::TestInit()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	int g_constraintSolverType = 0;

	btMultiBodyConstraintSolver* sol;
	btMLCPSolverInterface* mlcp;
	switch (g_constraintSolverType)
	{
	case 0:
		sol = new btMultiBodyConstraintSolver;
		break;
	case 1:
		mlcp = new btSolveProjectedGaussSeidel();
		sol = new btMultiBodyMLCPConstraintSolver(mlcp);
		break;
	case 2:
		mlcp = new btDantzigSolver();
		sol = new btMultiBodyMLCPConstraintSolver(mlcp);
		break;
	default:
		mlcp = new btLemkeSolver();
		sol = new btMultiBodyMLCPConstraintSolver(mlcp);
		break;
	}

	m_solver = sol;

	//use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
	auto world = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration);
	m_dynamicsWorld = world;
	//	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 1e-3;

	///create a few basic rigid bodies
	btVector3 groundHalfExtents(50, 50, 50);
	btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);
	//groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, -50));
	ground = Matrix::CreateTranslation(-0.5 * Vector3::One)
		* Matrix::CreateScale(groundHalfExtents.getX(), groundHalfExtents.getY(), groundHalfExtents.getZ())
		* Matrix::CreateTranslation(0, 0, -50);

	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////

	bool damping = true;
	bool gyro = true;
	int numLinks = 5;
	bool spherical = true;  //set it ot false -to use 1DoF hinges instead of 3DoF sphericals
	bool multibodyOnly = false;
	bool canSleep = false;
	bool selfCollide = true;
	bool multibodyConstraint = false;
	btVector3 linkHalfExtents(0.05, 0.37, 0.1);
	btVector3 baseHalfExtents(0.05, 0.37, 0.1);

	btMultiBody* mbC = createFeatherstoneMultiBody_testMultiDof(world, numLinks, btVector3(-0.4f, 3.f, 0.f), linkHalfExtents, baseHalfExtents, spherical, g_floatingBase);
	//mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm

	mbC->setCanSleep(canSleep);
	mbC->setHasSelfCollision(selfCollide);
	mbC->setUseGyroTerm(gyro);
	//
	if (!damping)
	{
		mbC->setLinearDamping(0.f);
		mbC->setAngularDamping(0.f);
	}
	else
	{
		mbC->setLinearDamping(0.1f);
		mbC->setAngularDamping(0.9f);
	}
	//
	m_dynamicsWorld->setGravity(btVector3(0, 0, -9.81));
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	//////////////////////////////////////////////
	if (numLinks > 0)
	{
		btScalar q0 = 45.f * SIMD_PI / 180.f;
		if (!spherical)
		{
			mbC->setJointPosMultiDof(0, &q0);
		}
		else
		{
			btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
			quat0.normalize();
			mbC->setJointPosMultiDof(0, quat0);
		}
	}
	///
	addColliders_testMultiDof(mbC, world, baseHalfExtents, linkHalfExtents);

	/////////////////////////////////////////////////////////////////
	btScalar groundHeight = -51.55;
	if (!multibodyOnly)
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, groundHeight, 0));
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body, 1, 1 + 2);  //,1,1+2);
	}
	/////////////////////////////////////////////////////////////////
	if (!multibodyOnly)
	{
		btVector3 halfExtents(.5, .5, .5);
		btBoxShape* colShape = new btBoxShape(halfExtents);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(
			btScalar(0.0),
			0.0,
			btScalar(0.0)));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_dynamicsWorld->addRigidBody(body);  //,1,1+2);

		if (multibodyConstraint)
		{
			btVector3 pointInA = -linkHalfExtents;
			//      btVector3 pointInB = halfExtents;
			btMatrix3x3 frameInA;
			btMatrix3x3 frameInB;
			frameInA.setIdentity();
			frameInB.setIdentity();
			btVector3 jointAxis(1.0, 0.0, 0.0);
			//btMultiBodySliderConstraint* p2p = new btMultiBodySliderConstraint(mbC,numLinks-1,body,pointInA,pointInB,frameInA,frameInB,jointAxis);
			btMultiBodyFixedConstraint* p2p = new btMultiBodyFixedConstraint(mbC, numLinks - 1, mbC, numLinks - 4, pointInA, pointInA, frameInA, frameInB);
			p2p->setMaxAppliedImpulse(2.0);
			m_dynamicsWorld->addMultiBodyConstraint(p2p);
		}
	}


	/////////////////////////////////////////////////////////////////
}

btMultiBody* Simulation::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating)
{
	//init the base
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape* pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;

	btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);

	btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);
	//	pMultiBody->setBaseVel(vel);

	//init the links
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape* pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	//////
	btScalar q0 = 0.f * SIMD_PI / 180.f;
	btQuaternion quat0(btVector3(0, 1, 0).normalized(), q0);
	quat0.normalize();
	/////

	for (int i = 0; i < numLinks; ++i)
	{
		if (!spherical)
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	}

	pMultiBody->finalizeMultiDof();

	///
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void Simulation::addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{
		//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
		btScalar quat[4] = { -world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w() };

		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
			col->setCollisionShape(box);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);

			pWorld->addCollisionObject(col, 2, 1 + 2);

			col->setFriction(friction);
			pMultiBody->setBaseCollider(col);
		}
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
		local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		btVector3 posr = local_origin[i + 1];
		//	float pos[4]={posr.x(),posr.y(),posr.z(),1};

		btScalar quat[4] = { -world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w() };

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		pWorld->addCollisionObject(col, 2, 1 + 2);

		pMultiBody->getLink(i).m_collider = col;
	}
}

Simulation::~Simulation()
{
	if (m_dynamicsWorld)
	{
		int i;
		for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
		{
			m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
		}

		for (i = m_dynamicsWorld->getNumMultiBodyConstraints() - 1; i >= 0; i--)
		{
			btMultiBodyConstraint* mbc = m_dynamicsWorld->getMultiBodyConstraint(i);
			m_dynamicsWorld->removeMultiBodyConstraint(mbc);
			delete mbc;
		}

		for (i = m_dynamicsWorld->getNumMultibodies() - 1; i >= 0; i--)
		{
			btMultiBody* mb = m_dynamicsWorld->getMultiBody(i);
			m_dynamicsWorld->removeMultiBody(mb);
			delete mb;
		}
		for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				delete body->getMotionState();
			}
			m_dynamicsWorld->removeCollisionObject(obj);
			delete obj;
		}
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		delete m_collisionShapes[j];
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;
}