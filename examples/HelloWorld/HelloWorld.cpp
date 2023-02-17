/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"

//#include "BulletDynamics/Dynamics/exDebugDDWorld.h"

#include "BulletDynamics/Dynamics/exCopyDDWorld.h"

#include <stdio.h>

/// This is a Hello World program for running a basic Bullet physics simulation

int main(int argc, char** argv)
{
	///-----includes_end-----

	int i;
	///-----initialization_start-----
	/// 
	
	printf("Part 0\n");
	btDefaultCollisionConstructionInfo constructionInfo;
	constructionInfo.m_defaultMaxPersistentManifoldPoolSize = 16;
	constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize = 16;

	printf("Part 3\n");

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration(constructionInfo);

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

//	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

//	exDebugDDWorld* dynamicsWorld = new exDebugDDWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	exCopyDDWorld* dynamicsWorld = new exCopyDDWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	


	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	///create a few basic rigid bodies

	//the ground is a cube of side 100 at position y = -56.
	//the sphere will hit it at y = -6, with center at -5
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

		collisionShapes.push_back(groundShape);

		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -56, 0));

		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}

	{
		//create a dynamic rigidbody

		//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		startTransform.setOrigin(btVector3(2, 10, 0));

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(body);
	}

	/// Do some simulation

	///-----stepsimulation_start-----
	for (i = 0; i < 150; i++)
	{
#ifdef SIMPLE_BULLET_STEPSIMULATION
		dynamicsWorld->stepSimulation(1.f / 60.f, 10);
#else
		btScalar timeStep = 1.f / 60.f;
		int maxSubSteps = 10;
		int numSimulationSubSteps =	dynamicsWorld->stepSimulationPart0(timeStep, maxSubSteps);
		if (numSimulationSubSteps)
		{
			int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;
			dynamicsWorld->stepSimulationPart1(timeStep, maxSubSteps);
			for (int i = 0; i < clampedSimulationSteps; i++)
			{
				dynamicsWorld->stepSimulationPart2(timeStep);
				dynamicsWorld->stepSimulationPart3(timeStep);
				dynamicsWorld->stepSimulationPart4();
				dynamicsWorld->stepSimulationPart5();
				dynamicsWorld->stepSimulationPart6(timeStep);
				dynamicsWorld->stepSimulationPart7(timeStep);
				dynamicsWorld->stepSimulationPart8(timeStep);
				dynamicsWorld->stepSimulationPart9(timeStep);
			}
		}
		else
		{
			dynamicsWorld->stepSimulationPart10();
		}
		dynamicsWorld->stepSimulationPart11();

#endif
		exCopyDDWorld::IslandData * island_data_pt = nullptr;
		btAlignedObjectArray<btPersistentManifold*> * predictive_manifolds_data = dynamicsWorld->getPredictiveManifolds();

		island_data_pt = dynamicsWorld->getIslandCallbackDataPt();
		int island_data_cnt = island_data_pt->m_savedpoint.size();

		if (island_data_cnt)
		{
			for (int j = 0; j < island_data_cnt; j++)
			{
				btVector3 ptA = island_data_pt->m_savedpoint[j].getPositionWorldOnA();
				printf("point[%d]: %f; %f; %f\n", j, ptA.x(), ptA.y(), ptA.z());

			}
		}
		btConstraintArray * tmp_storage_pt = dynamicsWorld->getTmpSeqImplStorage();
		int tmp_storage_cnt = tmp_storage_pt->size();
		if (tmp_storage_cnt)
		{
			for (int j = 0; j < tmp_storage_cnt; j++)
			{
				btSolverConstraint val = tmp_storage_pt->at(j);
				printf("some\n");
			}
		}
		dynamicsWorld->clearTmpSeqImplStorage();
		dynamicsWorld->clearIslandCallbackData();
	
		if (predictive_manifolds_data)
			printf("Get predictive manifolds\n");

		//print positions of all objects
		for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			btTransform trans;
			if (body && body->getMotionState())
			{
				body->getMotionState()->getWorldTransform(trans);
			}
			else
			{
				trans = obj->getWorldTransform();
			}
			printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
		}
	}

	///-----stepsimulation_end-----

	//cleanup in the reverse order of creation/initialization

	///-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();

	printf("End\n");
	return 0;
}
