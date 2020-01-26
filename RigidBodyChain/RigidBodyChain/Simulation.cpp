#include "Simulation.h"

void Simulation::Init()
{
	time = 0;
	simulationSpeed = 1;
	paused = true;
	delta_time = 1.0 / 60;


	cubes.resize(2);
	poles.resize(5);

	InitBullet();

	cubes.resize(m_dynamicsWorld->getMultiBody(0)->getNumLinks());
	UpdateMatrices();
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

void Simulation::InitBullet()
{
	InitWorld();
	InitTerrain();
	InitMultiBody();
	InitHandlers();
}
void Simulation::InitWorld()
{
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();
	m_solver = new btMultiBodyConstraintSolver;

	//use btMultiBodyDynamicsWorld for Featherstone btMultiBody support
	m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
	m_dynamicsWorld->getSolverInfo().m_globalCfm = 1e-3;
}
void Simulation::InitTerrain()
{
	btVector3 groundHalfExtents(50, 50, 50);
	btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, -groundHalfExtents.getZ()));
	groundMatrix = Matrix::CreateTranslation(-0.5 * Vector3::One) * Matrix::CreateScale(2)
		* Matrix::CreateScale(groundHalfExtents.getX(), groundHalfExtents.getY(), groundHalfExtents.getZ())
		* Matrix::CreateTranslation(0, 0, -groundHalfExtents.getZ());

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, groundShape, { 0, 0, 0 });
	m_dynamicsWorld->addRigidBody(new btRigidBody(rbInfo), 1, 1 + 2);
}

void Simulation::InitMultiBody()
{
	int numLinks = cubes.size() + poles.size();

	btMultiBody* multiBody = InitEmptyMultiBody(numLinks);
	InitMultiBodyLinks(multiBody);

	multiBody->finalizeMultiDof();
	m_dynamicsWorld->addMultiBody(multiBody);

	InitMultiBodyLinksColliders(multiBody);
}
btMultiBody* Simulation::InitEmptyMultiBody(int numLinks)
{
	//init the base
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;
	btCollisionShape* pTempBox = new btBoxShape(halfSize);//?
	pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
	delete pTempBox;

	btMultiBody* multiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, false, false);

	multiBody->setCanSleep(false);
	multiBody->setHasSelfCollision(true);
	multiBody->setUseGyroTerm(true);
	multiBody->setLinearDamping(0.1f);
	multiBody->setAngularDamping(0.9f);

	btVector3 basePosition(0.f, 0.f, 5.f);
	multiBody->setBasePos(basePosition);
	multiBody->setWorldToBaseRot(btQuaternion::getIdentity());

	return multiBody;

}
void Simulation::InitMultiBodyLinks(btMultiBody* multiBody)
{
	//init the links
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);
	btCollisionShape* pTempBoxLink = new btBoxShape(halfSize);
	pTempBoxLink->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBoxLink;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -halfSize[1] * 2.f, 0);
	btVector3 currentPivotToCurrentCom(0, -halfSize[1], 0);
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;

	int numLinks = multiBody->getNumLinks();
	for (int i = 0; i < numLinks; ++i)
	{
		if (i == 5)
		{
			btVector3 currentPivotToCurrentCom(0, -halfSize[1], 0);
			btVector3 parentComToCurrentPivot(0, 0, 0);
			multiBody->setupSpherical(i, linkMass, linkInertiaDiag, 2, btQuaternion(0.f, 0.f, 0.8509035f, 0.525322f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
		}
		else
		{
			multiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
		}
	}
}
void Simulation::InitMultiBodyLinksColliders(btMultiBody* pMultiBody)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{
		btScalar quat[4] = { -world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w() };

		if (1)
		{
			btCollisionShape* box = new btBoxShape(halfSize);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
			col->setCollisionShape(box);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);

			m_dynamicsWorld->addCollisionObject(col, 2, 1 + 2);

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

		btScalar quat[4] = { -world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w() };

		btCollisionShape* box = new btBoxShape(halfSize);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		m_dynamicsWorld->addCollisionObject(col, 2, 1 + 2);

		pMultiBody->getLink(i).m_collider = col;
	}
}

void Simulation::InitHandlers()
{
	auto boxEnd = halfSize.y();
	handle1 = AddHandle(0, { 0,boxEnd,0 });
	handle2 = AddHandle(4, { 0,-boxEnd,0 });
}
btMultiBodyPoint2Point* Simulation::AddHandle(int idx, btVector3 pivot)
{
	auto multiCol = m_dynamicsWorld->getMultiBody(0)->getLink(idx).m_collider;
	multiCol->m_multiBody->setCanSleep(false);//?

	auto globalPos = multiCol->getWorldTransform().getOrigin();
	btMultiBodyPoint2Point* p2p = new btMultiBodyPoint2Point(multiCol->m_multiBody, multiCol->m_link, 0, pivot, globalPos);
	p2p->setMaxAppliedImpulse(2);//?
	m_dynamicsWorld->addMultiBodyConstraint(p2p);

	return p2p;
}
void Simulation::UpdateMatrices()
{
	auto multiBody = m_dynamicsWorld->getMultiBody(0);

	for (int i = 0; i < multiBody->getNumLinks(); i++)
	{
		auto transform = multiBody->getLink(i).m_collider->getWorldTransform();
		auto r = transform.getRotation();
		auto p = transform.getOrigin();
		cubes[i] = Matrix::CreateTranslation(-0.5 * Vector3::One) * Matrix::CreateScale(2)
			* Matrix::CreateScale(Vector3((float)halfSize.x(), (float)halfSize.y(), (float)halfSize.z()))
			* Matrix::CreateFromQuaternion({ (float)r.getX(),(float)r.getY(),(float)r.getZ(),(float)r.getW() })
			* Matrix::CreateTranslation({ (float)p.getX(),(float)p.getY(),(float)p.getZ() });
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