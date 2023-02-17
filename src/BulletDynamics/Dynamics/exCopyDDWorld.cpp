/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "exCopyDDWorld.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btQuickprof.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMotionState.h"

#include "LinearMath/btSerializer.h"

#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"

#if 0
btAlignedObjectArray<btVector3> debugContacts;
btAlignedObjectArray<btVector3> debugNormals;
int startHit=2;
int firstHit=startHit;
#endif

#ifdef BT_DEBUG
#include <stdio.h>
#ifndef EMBT_PRINT
#define EMBT_PRINT(x) printf(x)
#endif
#else
#ifndef EMBT_PRINT
#define EMBT_PRINT(x)
#endif
#endif

SIMD_FORCE_INLINE int btGetConstraintIslandId(const btTypedConstraint* lhs)
{
	int islandId;

	const btCollisionObject& rcolObj0 = lhs->getRigidBodyA();
	const btCollisionObject& rcolObj1 = lhs->getRigidBodyB();
	islandId = rcolObj0.getIslandTag() >= 0 ? rcolObj0.getIslandTag() : rcolObj1.getIslandTag();
	return islandId;
}

class btSortConstraintOnIslandPredicate
{
public:
	bool operator()(const btTypedConstraint* lhs, const btTypedConstraint* rhs) const
	{
		int rIslandId0, lIslandId0;
		rIslandId0 = btGetConstraintIslandId(rhs);
		lIslandId0 = btGetConstraintIslandId(lhs);
		return lIslandId0 < rIslandId0;
	}
};

struct InplaceSolverIslandCallback : public btSimulationIslandManager::IslandCallback
{
	btContactSolverInfo* m_solverInfo;
	btConstraintSolver* m_solver;
	btTypedConstraint** m_sortedConstraints;
	int m_numConstraints;
	btIDebugDraw* m_debugDrawer;
	btDispatcher* m_dispatcher;

	btAlignedObjectArray<btCollisionObject*> m_bodies;
	btAlignedObjectArray<btPersistentManifold*> m_manifolds;
	btAlignedObjectArray<btTypedConstraint*> m_constraints;


	exCopyDDWorld::IslandData m_island_data;

	InplaceSolverIslandCallback(
		btConstraintSolver* solver,
		btStackAlloc* stackAlloc,
		btDispatcher* dispatcher)
		: m_solverInfo(NULL),
		  m_solver(solver),
		  m_sortedConstraints(NULL),
		  m_numConstraints(0),
		  m_debugDrawer(NULL),
		  m_dispatcher(dispatcher)
	{
	}

	InplaceSolverIslandCallback& operator=(InplaceSolverIslandCallback& other)
	{
		btAssert(0);
		(void)other;
		return *this;
	}

	SIMD_FORCE_INLINE void setup(btContactSolverInfo* solverInfo, btTypedConstraint** sortedConstraints, int numConstraints, btIDebugDraw* debugDrawer)
	{
		btAssert(solverInfo);
		m_solverInfo = solverInfo;
		m_sortedConstraints = sortedConstraints;
		m_numConstraints = numConstraints;
		m_debugDrawer = debugDrawer;
		m_bodies.resize(0);
		m_manifolds.resize(0);
		m_constraints.resize(0);
	}

	virtual void processIsland(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifolds, int numManifolds, int islandId)
	{
		if (numManifolds && numBodies)
		{
			for (int i = 0; i < numManifolds; i++)
			{
				auto man = manifolds[i];
				int num_contacts = man->getNumContacts();
				for (int j = 0; j < num_contacts; j++)
				{
					//printf("Point[%d]\n", j);
					btManifoldPoint mpt = man->getContactPoint(j);
					/*btVector3 pt = man->getContactPoint(j).getPositionWorldOnA();
					printf("A: %f, %f, %f\n",  pt.x(), pt.y(), pt.z() );
					pt = man->getContactPoint(j).getPositionWorldOnB();
					printf("B: %f, %f, %f\n",  pt.x(), pt.y(), pt.z() );*/

					m_island_data.m_savedpoint.push_back(mpt);
					
				}
			}
		}
		if (islandId < 0)
		{
			///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
			m_solver->solveGroup(bodies, numBodies, manifolds, numManifolds, &m_sortedConstraints[0], m_numConstraints, *m_solverInfo, m_debugDrawer, m_dispatcher);
		}
		else
		{
			//also add all non-contact constraints/joints for this island
			btTypedConstraint** startConstraint = 0;
			int numCurConstraints = 0;
			int i;

			//find the first constraint for this island
			for (i = 0; i < m_numConstraints; i++)
			{
				if (btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
				{
					startConstraint = &m_sortedConstraints[i];
					break;
				}
			}
			//count the number of constraints in this island
			for (; i < m_numConstraints; i++)
			{
				if (btGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
				{
					numCurConstraints++;
				}
			}

			if (m_solverInfo->m_minimumSolverBatchSize <= 1)
			{
				m_solver->solveGroup(bodies, numBodies, manifolds, numManifolds, startConstraint, numCurConstraints, *m_solverInfo, m_debugDrawer, m_dispatcher);
			}
			else
			{
				for (i = 0; i < numBodies; i++)
					m_bodies.push_back(bodies[i]);
				for (i = 0; i < numManifolds; i++)
					m_manifolds.push_back(manifolds[i]);
				for (i = 0; i < numCurConstraints; i++)
					m_constraints.push_back(startConstraint[i]);
				if ((m_constraints.size() + m_manifolds.size()) > m_solverInfo->m_minimumSolverBatchSize)
				{
					processConstraints();
				}
				else
				{
					//printf("deferred\n");
				}
			}
		}
	}
	void processConstraints()
	{
		btCollisionObject** bodies = m_bodies.size() ? &m_bodies[0] : 0;
		btPersistentManifold** manifold = m_manifolds.size() ? &m_manifolds[0] : 0;
		btTypedConstraint** constraints = m_constraints.size() ? &m_constraints[0] : 0;

		m_solver->solveGroup(bodies, m_bodies.size(), manifold, m_manifolds.size(), constraints, m_constraints.size(), *m_solverInfo, m_debugDrawer, m_dispatcher);
		m_bodies.resize(0);
		m_manifolds.resize(0);
		m_constraints.resize(0);
	}
};

exCopyDDWorld::exCopyDDWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration)
	: btDynamicsWorld(dispatcher, pairCache, collisionConfiguration),
	  m_sortedConstraints(),
	  m_solverIslandCallback(NULL),
	  m_constraintSolver(constraintSolver),
	  m_gravity(0, -10, 0),
	  m_localTime(0),
	  m_fixedTimeStep(0),
	  m_synchronizeAllMotionStates(false),
	  m_applySpeculativeContactRestitution(false),
	  m_profileTimings(0),
	  m_latencyMotionStateInterpolation(true)

{
	TmpDataStorage = new exConstSolvDataStorage(&TmpDataStorageData[0], EX_COPY_DDWORLD_DATADRAW_STORAGE_SIZE);
	if (!m_constraintSolver)
	{
		void* mem = btAlignedAlloc(sizeof(btSequentialImpulseConstraintSolver), 16);
		m_constraintSolver = new (mem) btSequentialImpulseConstraintSolver;
		m_ownsConstraintSolver = true;
	}
	else
	{
		m_ownsConstraintSolver = false;
	}

	{
		void* mem = btAlignedAlloc(sizeof(btSimulationIslandManager), 16);
		m_islandManager = new (mem) btSimulationIslandManager();
	}

	m_ownsIslandManager = true;

	{
		void* mem = btAlignedAlloc(sizeof(InplaceSolverIslandCallback), 16);
		m_solverIslandCallback = new (mem) InplaceSolverIslandCallback(m_constraintSolver, 0, dispatcher);
	}
}

exCopyDDWorld::~exCopyDDWorld()
{
	//only delete it when we created it
	if (m_ownsIslandManager)
	{
		m_islandManager->~btSimulationIslandManager();
		btAlignedFree(m_islandManager);
	}
	if (m_solverIslandCallback)
	{
		m_solverIslandCallback->~InplaceSolverIslandCallback();
		btAlignedFree(m_solverIslandCallback);
	}
	if (m_ownsConstraintSolver)
	{
		m_constraintSolver->~btConstraintSolver();
		btAlignedFree(m_constraintSolver);
	}
}

void exCopyDDWorld::saveKinematicState(btScalar timeStep)
{
	///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
	///to switch status _after_ adding kinematic objects to the world
	///fix it for Bullet 3.x release
	for (int i = 0; i < m_collisionObjects.size(); i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body && body->getActivationState() != ISLAND_SLEEPING)
		{
			if (body->isKinematicObject())
			{
				//to calculate velocities next frame
				body->saveKinematicState(timeStep);
			}
		}
	}
}

void exCopyDDWorld::debugDrawWorld()
{
	BT_PROFILE("debugDrawWorld");

	btCollisionWorld::debugDrawWorld();

	bool drawConstraints = false;
	if (getDebugDrawer())
	{
		int mode = getDebugDrawer()->getDebugMode();
		if (mode & (btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if (drawConstraints)
	{
		for (int i = getNumConstraints() - 1; i >= 0; i--)
		{
			btTypedConstraint* constraint = getConstraint(i);
			debugDrawConstraint(constraint);
		}
	}

	if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawNormals)))
	{
		int i;

		if (getDebugDrawer() && getDebugDrawer()->getDebugMode())
		{
			for (i = 0; i < m_actions.size(); i++)
			{
				m_actions[i]->debugDraw(m_debugDrawer);
			}
		}
	}
	if (getDebugDrawer())
		getDebugDrawer()->flushLines();
}

void exCopyDDWorld::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		//need to check if next line is ok
		//it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
		body->clearForces();
	}
}

///apply gravity, call this once per timestep
void exCopyDDWorld::applyGravity()
{
	///@todo: iterate over awake simulation islands!
	for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive())
		{
			body->applyGravity();
		}
	}
}

void exCopyDDWorld::synchronizeSingleMotionState(btRigidBody* body)
{
	btAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we need to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		///@todo: add 'dirty' flag
		//if (body->getActivationState() != ISLAND_SLEEPING)
		{
			btTransform interpolatedTransform;
			btTransformUtil::integrateTransform(body->getInterpolationWorldTransform(),
												body->getInterpolationLinearVelocity(), body->getInterpolationAngularVelocity(),
												(m_latencyMotionStateInterpolation && m_fixedTimeStep) ? m_localTime - m_fixedTimeStep : m_localTime * body->getHitFraction(),
												interpolatedTransform);
			body->getMotionState()->setWorldTransform(interpolatedTransform);
		}
	}
}

void exCopyDDWorld::synchronizeMotionStates()
{
	//	BT_PROFILE("synchronizeMotionStates");
	if (m_synchronizeAllMotionStates)
	{
		//iterate  over all collision objects
		for (int i = 0; i < m_collisionObjects.size(); i++)
		{
			btCollisionObject* colObj = m_collisionObjects[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body)
				synchronizeSingleMotionState(body);
		}
	}
	else
	{
		//iterate over all active rigid bodies
		for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
		{
			btRigidBody* body = m_nonStaticRigidBodies[i];
			if (body->isActive())
				synchronizeSingleMotionState(body);
		}
	}
}

int exCopyDDWorld::stepSimulation(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
	printf("stepSimulation\n");
	startProfiling(timeStep);

	int numSimulationSubSteps = 0;

	if (maxSubSteps)
	{
		//fixed timestep with interpolation
		m_fixedTimeStep = fixedTimeStep;
		m_localTime += timeStep;
		if (m_localTime >= fixedTimeStep)
		{
			numSimulationSubSteps = int(m_localTime / fixedTimeStep);
			m_localTime -= numSimulationSubSteps * fixedTimeStep;
		}
	}
	else
	{
		//variable timestep
		fixedTimeStep = timeStep;
		m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
		m_fixedTimeStep = 0;
		if (btFuzzyZero(timeStep))
		{
			numSimulationSubSteps = 0;
			maxSubSteps = 0;
		}
		else
		{
			numSimulationSubSteps = 1;
			maxSubSteps = 1;
		}
	}

	//process some debugging flags
	if (getDebugDrawer())
	{
		btIDebugDraw* debugDrawer = getDebugDrawer();
		gDisableDeactivation = (debugDrawer->getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0;
	}
	if (numSimulationSubSteps)
	{
		//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
		int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;

		saveKinematicState(fixedTimeStep * clampedSimulationSteps);

		applyGravity();

		printf("clamped simulation steps: %d\n", clampedSimulationSteps);

		for (int i = 0; i < clampedSimulationSteps; i++)
		{
			internalSingleStepSimulation(fixedTimeStep);
			synchronizeMotionStates();
		}
	}
	else
	{
		synchronizeMotionStates();
	}

	clearForces();

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif  //BT_NO_PROFILE



	return numSimulationSubSteps;
}


void exCopyDDWorld::internalSingleStepSimulation(btScalar timeStep)
{
	BT_PROFILE("internalSingleStepSimulation");

	if (0 != m_internalPreTickCallback)
	{
		(*m_internalPreTickCallback)(this, timeStep);
	}

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	btDispatcherInfo& dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	createPredictiveContacts(timeStep);

	///perform collision detection
	performDiscreteCollisionDetection();

	getManifoldDataToDraw(); //#Debug

	calculateSimulationIslands();

	getSolverInfo().m_timeStep = timeStep;

	///solve contact and other joint constraints
	solveConstraints(getSolverInfo());

	///CallbackTriggers();

	///integrate transforms

	integrateTransforms(timeStep);

	///update vehicle simulation
	updateActions(timeStep);

	updateActivationState(timeStep);

	if (0 != m_internalTickCallback)
	{
		(*m_internalTickCallback)(this, timeStep);
	}
}

void exCopyDDWorld::setGravity(const btVector3& gravity)
{
	m_gravity = gravity;
	for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY))
		{
			body->setGravity(gravity);
		}
	}
}

btVector3 exCopyDDWorld::getGravity() const
{
	return m_gravity;
}

void exCopyDDWorld::addCollisionObject(btCollisionObject* collisionObject, int collisionFilterGroup, int collisionFilterMask)
{
	btCollisionWorld::addCollisionObject(collisionObject, collisionFilterGroup, collisionFilterMask);
}

void exCopyDDWorld::removeCollisionObject(btCollisionObject* collisionObject)
{
	btRigidBody* body = btRigidBody::upcast(collisionObject);
	if (body)
		removeRigidBody(body);
	else
		btCollisionWorld::removeCollisionObject(collisionObject);
}

void exCopyDDWorld::removeRigidBody(btRigidBody* body)
{
	m_nonStaticRigidBodies.remove(body);
	btCollisionWorld::removeCollisionObject(body);
}

void exCopyDDWorld::addRigidBody(btRigidBody* body)
{
	EMBT_PRINT("Add rigit body\n");
	if (!body->isStaticOrKinematicObject() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY))
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		EMBT_PRINT("add collision obj\n");
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		}
		else
		{
			body->setActivationState(ISLAND_SLEEPING);
		}

		bool isDynamic = !(body->isStaticObject() || body->isKinematicObject());
		int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
		int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

		addCollisionObject(body, collisionFilterGroup, collisionFilterMask);
	}
	EMBT_PRINT("End rigit body\n");
}

void exCopyDDWorld::addRigidBody(btRigidBody* body, int group, int mask)
{
	if (!body->isStaticOrKinematicObject() && !(body->getFlags() & BT_DISABLE_WORLD_GRAVITY))
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		}
		else
		{
			body->setActivationState(ISLAND_SLEEPING);
		}
		addCollisionObject(body, group, mask);
	}
}

void exCopyDDWorld::updateActions(btScalar timeStep)
{
	BT_PROFILE("updateActions");

	for (int i = 0; i < m_actions.size(); i++)
	{
		m_actions[i]->updateAction(this, timeStep);
	}
}

void exCopyDDWorld::updateActivationState(btScalar timeStep)
{
	BT_PROFILE("updateActivationState");

	for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (body)
		{
			body->updateDeactivation(timeStep);

			if (body->wantsSleeping())
			{
				if (body->isStaticOrKinematicObject())
				{
					body->setActivationState(ISLAND_SLEEPING);
				}
				else
				{
					if (body->getActivationState() == ACTIVE_TAG)
						body->setActivationState(WANTS_DEACTIVATION);
					if (body->getActivationState() == ISLAND_SLEEPING)
					{
						body->setAngularVelocity(btVector3(0, 0, 0));
						body->setLinearVelocity(btVector3(0, 0, 0));
					}
				}
			}
			else
			{
				if (body->getActivationState() != DISABLE_DEACTIVATION)
					body->setActivationState(ACTIVE_TAG);
			}
		}
	}
}

void exCopyDDWorld::addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies)
{
	m_constraints.push_back(constraint);
	//Make sure the two bodies of a type constraint are different (possibly add this to the btTypedConstraint constructor?)
	btAssert(&constraint->getRigidBodyA() != &constraint->getRigidBodyB());

	if (disableCollisionsBetweenLinkedBodies)
	{
		constraint->getRigidBodyA().addConstraintRef(constraint);
		constraint->getRigidBodyB().addConstraintRef(constraint);
	}
}

void exCopyDDWorld::removeConstraint(btTypedConstraint* constraint)
{
	m_constraints.remove(constraint);
	constraint->getRigidBodyA().removeConstraintRef(constraint);
	constraint->getRigidBodyB().removeConstraintRef(constraint);
}

void exCopyDDWorld::addAction(btActionInterface* action)
{
	m_actions.push_back(action);
}

void exCopyDDWorld::removeAction(btActionInterface* action)
{
	m_actions.remove(action);
}

void exCopyDDWorld::addVehicle(btActionInterface* vehicle)
{
	addAction(vehicle);
}

void exCopyDDWorld::removeVehicle(btActionInterface* vehicle)
{
	removeAction(vehicle);
}

void exCopyDDWorld::addCharacter(btActionInterface* character)
{
	addAction(character);
}

void exCopyDDWorld::removeCharacter(btActionInterface* character)
{
	removeAction(character);
}

void exCopyDDWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	BT_PROFILE("solveConstraints");

	m_sortedConstraints.resize(m_constraints.size());
	int i;
	for (i = 0; i < getNumConstraints(); i++)
	{
		m_sortedConstraints[i] = m_constraints[i];
	}

	//	btAssert(0);

	m_sortedConstraints.quickSort(btSortConstraintOnIslandPredicate());

	btTypedConstraint** constraintsPtr = getNumConstraints() ? &m_sortedConstraints[0] : 0;

	m_solverIslandCallback->setup(&solverInfo, constraintsPtr, m_sortedConstraints.size(), getDebugDrawer());
	m_constraintSolver->prepareSolve(getCollisionWorld()->getNumCollisionObjects(), getCollisionWorld()->getDispatcher()->getNumManifolds());

	/// solve all the constraints for this island
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(), getCollisionWorld(), m_solverIslandCallback);

	m_solverIslandCallback->processConstraints();

	m_constraintSolver->allSolved(solverInfo, m_debugDrawer);
}

void exCopyDDWorld::calculateSimulationIslands()
{
	BT_PROFILE("calculateSimulationIslands");

	getSimulationIslandManager()->updateActivationState(getCollisionWorld(), getCollisionWorld()->getDispatcher());

	{
		//merge islands based on speculative contact manifolds too
		for (int i = 0; i < this->m_predictiveManifolds.size(); i++)
		{
			btPersistentManifold* manifold = m_predictiveManifolds[i];

			const btCollisionObject* colObj0 = manifold->getBody0();
			const btCollisionObject* colObj1 = manifold->getBody1();

			if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
				((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
			{
				getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
			}
		}
	}

	{
		int i;
		int numConstraints = int(m_constraints.size());
		for (i = 0; i < numConstraints; i++)
		{
			btTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled())
			{
				const btRigidBody* colObj0 = &constraint->getRigidBodyA();
				const btRigidBody* colObj1 = &constraint->getRigidBodyB();

				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
				{
					getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(), (colObj1)->getIslandTag());
				}
			}
		}
	}

	//Store the island id in each body
	getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());
}

class btClosestNotMeConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback
{
public:
	btCollisionObject* m_me;
	btScalar m_allowedPenetration;
	btOverlappingPairCache* m_pairCache;
	btDispatcher* m_dispatcher;

public:
	btClosestNotMeConvexResultCallback(btCollisionObject* me, const btVector3& fromA, const btVector3& toA, btOverlappingPairCache* pairCache, btDispatcher* dispatcher) : btCollisionWorld::ClosestConvexResultCallback(fromA, toA),
																																										   m_me(me),
																																										   m_allowedPenetration(0.0f),
																																										   m_pairCache(pairCache),
																																										   m_dispatcher(dispatcher)
	{
	}

	virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0f;

		//ignore result if there is no contact response
		if (!convexResult.m_hitCollisionObject->hasContactResponse())
			return 1.0f;

		btVector3 linVelA, linVelB;
		linVelA = m_convexToWorld - m_convexFromWorld;
		linVelB = btVector3(0, 0, 0);  //toB.getOrigin()-fromB.getOrigin();

		btVector3 relativeVelocity = (linVelA - linVelB);
		//don't report time of impact for motion away from the contact normal (or causes minor penetration)
		if (convexResult.m_hitNormalLocal.dot(relativeVelocity) >= -m_allowedPenetration)
			return 1.f;

		return ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
	}

	virtual bool needsCollision(btBroadphaseProxy* proxy0) const
	{
		//don't collide with itself
		if (proxy0->m_clientObject == m_me)
			return false;

		///don't do CCD when the collision filters are not matching
		if (!ClosestConvexResultCallback::needsCollision(proxy0))
			return false;
		if (m_pairCache->getOverlapFilterCallback()) {
			btBroadphaseProxy* proxy1 = m_me->getBroadphaseHandle();
			bool collides = m_pairCache->needsBroadphaseCollision(proxy0, proxy1);
			if (!collides)
			{
				return false;
			}
		}

		btCollisionObject* otherObj = (btCollisionObject*)proxy0->m_clientObject;

		if (!m_dispatcher->needsCollision(m_me, otherObj))
			return false;

		//call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
		if (m_dispatcher->needsResponse(m_me, otherObj))
		{
#if 0
			///don't do CCD when there are already contact points (touching contact/penetration)
			btAlignedObjectArray<btPersistentManifold*> manifoldArray;
			btBroadphasePair* collisionPair = m_pairCache->findPair(m_me->getBroadphaseHandle(),proxy0);
			if (collisionPair)
			{
				if (collisionPair->m_algorithm)
				{
					manifoldArray.resize(0);
					collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
					for (int j=0;j<manifoldArray.size();j++)
					{
						btPersistentManifold* manifold = manifoldArray[j];
						if (manifold->getNumContacts()>0)
							return false;
					}
				}
			}
#endif
			return true;
		}

		return false;
	}
};

///internal debugging variable. this value shouldn't be too high
int gNumClampedCcdMotions = 0;

void exCopyDDWorld::createPredictiveContactsInternal(btRigidBody** bodies, int numBodies, btScalar timeStep)
{
	btTransform predictedTrans;
	for (int i = 0; i < numBodies; i++)
	{
		btRigidBody* body = bodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{
			body->predictIntegratedTransform(timeStep, predictedTrans);

			btScalar squareMotion = (predictedTrans.getOrigin() - body->getWorldTransform().getOrigin()).length2();

			if (getDispatchInfo().m_useContinuous && body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion)
			{
				BT_PROFILE("predictive convexSweepTest");
				if (body->getCollisionShape()->isConvex())
				{
					gNumClampedCcdMotions++;
#ifdef PREDICTIVE_CONTACT_USE_STATIC_ONLY
					class StaticOnlyCallback : public btClosestNotMeConvexResultCallback
					{
					public:
						StaticOnlyCallback(btCollisionObject* me, const btVector3& fromA, const btVector3& toA, btOverlappingPairCache* pairCache, btDispatcher* dispatcher) : btClosestNotMeConvexResultCallback(me, fromA, toA, pairCache, dispatcher)
						{
						}

						virtual bool needsCollision(btBroadphaseProxy* proxy0) const
						{
							btCollisionObject* otherObj = (btCollisionObject*)proxy0->m_clientObject;
							if (!otherObj->isStaticOrKinematicObject())
								return false;
							return btClosestNotMeConvexResultCallback::needsCollision(proxy0);
						}
					};

					StaticOnlyCallback sweepResults(body, body->getWorldTransform().getOrigin(), predictedTrans.getOrigin(), getBroadphase()->getOverlappingPairCache(), getDispatcher());
#else
					btClosestNotMeConvexResultCallback sweepResults(body, body->getWorldTransform().getOrigin(), predictedTrans.getOrigin(), getBroadphase()->getOverlappingPairCache(), getDispatcher());
#endif
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					btSphereShape tmpSphere(body->getCcdSweptSphereRadius());  //btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					sweepResults.m_allowedPenetration = getDispatchInfo().m_allowedCcdPenetration;

					sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask = body->getBroadphaseProxy()->m_collisionFilterMask;
					btTransform modifiedPredictedTrans = predictedTrans;
					modifiedPredictedTrans.setBasis(body->getWorldTransform().getBasis());

					convexSweepTest(&tmpSphere, body->getWorldTransform(), modifiedPredictedTrans, sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f))
					{
						btVector3 distVec = (predictedTrans.getOrigin() - body->getWorldTransform().getOrigin()) * sweepResults.m_closestHitFraction;
						btScalar distance = distVec.dot(-sweepResults.m_hitNormalWorld);

						btMutexLock(&m_predictiveManifoldsMutex);
						btPersistentManifold* manifold = m_dispatcher1->getNewManifold(body, sweepResults.m_hitCollisionObject);
						m_predictiveManifolds.push_back(manifold);
						btMutexUnlock(&m_predictiveManifoldsMutex);

						btVector3 worldPointB = body->getWorldTransform().getOrigin() + distVec;
						btVector3 localPointB = sweepResults.m_hitCollisionObject->getWorldTransform().inverse() * worldPointB;

						btManifoldPoint newPoint(btVector3(0, 0, 0), localPointB, sweepResults.m_hitNormalWorld, distance);

						bool isPredictive = true;
						int index = manifold->addManifoldPoint(newPoint, isPredictive);
						btManifoldPoint& pt = manifold->getContactPoint(index);
						pt.m_combinedRestitution = 0;
						pt.m_combinedFriction = gCalculateCombinedFrictionCallback(body, sweepResults.m_hitCollisionObject);
						pt.m_positionWorldOnA = body->getWorldTransform().getOrigin();
						pt.m_positionWorldOnB = worldPointB;
					}
				}
			}
		}
	}
}

void exCopyDDWorld::releasePredictiveContacts()
{
	BT_PROFILE("release predictive contact manifolds");

	for (int i = 0; i < m_predictiveManifolds.size(); i++)
	{
		btPersistentManifold* manifold = m_predictiveManifolds[i];
		this->m_dispatcher1->releaseManifold(manifold);
	}
	printf("Predictive manifolds clear\n");
	m_predictiveManifolds.clear();
}

void exCopyDDWorld::createPredictiveContacts(btScalar timeStep)
{
	BT_PROFILE("createPredictiveContacts");
	releasePredictiveContacts();
	if (m_nonStaticRigidBodies.size() > 0)
	{
		createPredictiveContactsInternal(&m_nonStaticRigidBodies[0], m_nonStaticRigidBodies.size(), timeStep);
	}
}

void exCopyDDWorld::integrateTransformsInternal(btRigidBody** bodies, int numBodies, btScalar timeStep)
{
	btTransform predictedTrans;
	for (int i = 0; i < numBodies; i++)
	{
		btRigidBody* body = bodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{
			body->predictIntegratedTransform(timeStep, predictedTrans);

			btScalar squareMotion = (predictedTrans.getOrigin() - body->getWorldTransform().getOrigin()).length2();

			if (getDispatchInfo().m_useContinuous && body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion)
			{
				BT_PROFILE("CCD motion clamping");
				if (body->getCollisionShape()->isConvex())
				{
					gNumClampedCcdMotions++;
#ifdef USE_STATIC_ONLY
					class StaticOnlyCallback : public btClosestNotMeConvexResultCallback
					{
					public:
						StaticOnlyCallback(btCollisionObject* me, const btVector3& fromA, const btVector3& toA, btOverlappingPairCache* pairCache, btDispatcher* dispatcher) : btClosestNotMeConvexResultCallback(me, fromA, toA, pairCache, dispatcher)
						{
						}

						virtual bool needsCollision(btBroadphaseProxy* proxy0) const
						{
							btCollisionObject* otherObj = (btCollisionObject*)proxy0->m_clientObject;
							if (!otherObj->isStaticOrKinematicObject())
								return false;
							return btClosestNotMeConvexResultCallback::needsCollision(proxy0);
						}
					};

					StaticOnlyCallback sweepResults(body, body->getWorldTransform().getOrigin(), predictedTrans.getOrigin(), getBroadphase()->getOverlappingPairCache(), getDispatcher());
#else
					btClosestNotMeConvexResultCallback sweepResults(body, body->getWorldTransform().getOrigin(), predictedTrans.getOrigin(), getBroadphase()->getOverlappingPairCache(), getDispatcher());
#endif
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					btSphereShape tmpSphere(body->getCcdSweptSphereRadius());  //btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					sweepResults.m_allowedPenetration = getDispatchInfo().m_allowedCcdPenetration;

					sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask = body->getBroadphaseProxy()->m_collisionFilterMask;
					btTransform modifiedPredictedTrans = predictedTrans;
					modifiedPredictedTrans.setBasis(body->getWorldTransform().getBasis());

					convexSweepTest(&tmpSphere, body->getWorldTransform(), modifiedPredictedTrans, sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f))
					{
						//printf("clamped integration to hit fraction = %f\n",fraction);
						body->setHitFraction(sweepResults.m_closestHitFraction);
						body->predictIntegratedTransform(timeStep * body->getHitFraction(), predictedTrans);
						body->setHitFraction(0.f);
						body->proceedToTransform(predictedTrans);

#if 0
						btVector3 linVel = body->getLinearVelocity();

						btScalar maxSpeed = body->getCcdMotionThreshold()/getSolverInfo().m_timeStep;
						btScalar maxSpeedSqr = maxSpeed*maxSpeed;
						if (linVel.length2()>maxSpeedSqr)
						{
							linVel.normalize();
							linVel*= maxSpeed;
							body->setLinearVelocity(linVel);
							btScalar ms2 = body->getLinearVelocity().length2();
							body->predictIntegratedTransform(timeStep, predictedTrans);

							btScalar sm2 = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin()).length2();
							btScalar smt = body->getCcdSquareMotionThreshold();
							printf("sm2=%f\n",sm2);
						}
#else

						//don't apply the collision response right now, it will happen next frame
						//if you really need to, you can uncomment next 3 lines. Note that is uses zero restitution.
						//btScalar appliedImpulse = 0.f;
						//btScalar depth = 0.f;
						//appliedImpulse = resolveSingleCollision(body,(btCollisionObject*)sweepResults.m_hitCollisionObject,sweepResults.m_hitPointWorld,sweepResults.m_hitNormalWorld,getSolverInfo(), depth);

#endif

						continue;
					}
				}
			}

			body->proceedToTransform(predictedTrans);
		}
	}
}

void exCopyDDWorld::integrateTransforms(btScalar timeStep)
{
	BT_PROFILE("integrateTransforms");
	if (m_nonStaticRigidBodies.size() > 0)
	{
		integrateTransformsInternal(&m_nonStaticRigidBodies[0], m_nonStaticRigidBodies.size(), timeStep);
	}

	///this should probably be switched on by default, but it is not well tested yet
	if (m_applySpeculativeContactRestitution)
	{
		BT_PROFILE("apply speculative contact restitution");
		for (int i = 0; i < m_predictiveManifolds.size(); i++)
		{
			btPersistentManifold* manifold = m_predictiveManifolds[i];
			btRigidBody* body0 = btRigidBody::upcast((btCollisionObject*)manifold->getBody0());
			btRigidBody* body1 = btRigidBody::upcast((btCollisionObject*)manifold->getBody1());

			for (int p = 0; p < manifold->getNumContacts(); p++)
			{
				const btManifoldPoint& pt = manifold->getContactPoint(p);
				btScalar combinedRestitution = gCalculateCombinedRestitutionCallback(body0, body1);

				if (combinedRestitution > 0 && pt.m_appliedImpulse != 0.f)
				//if (pt.getDistance()>0 && combinedRestitution>0 && pt.m_appliedImpulse != 0.f)
				{
					btVector3 imp = -pt.m_normalWorldOnB * pt.m_appliedImpulse * combinedRestitution;

					const btVector3& pos1 = pt.getPositionWorldOnA();
					const btVector3& pos2 = pt.getPositionWorldOnB();

					btVector3 rel_pos0 = pos1 - body0->getWorldTransform().getOrigin();
					btVector3 rel_pos1 = pos2 - body1->getWorldTransform().getOrigin();

					if (body0)
						body0->applyImpulse(imp, rel_pos0);
					if (body1)
						body1->applyImpulse(-imp, rel_pos1);
				}
			}
		}
	}
}

void exCopyDDWorld::predictUnconstraintMotion(btScalar timeStep)
{
	BT_PROFILE("predictUnconstraintMotion");
	for (int i = 0; i < m_nonStaticRigidBodies.size(); i++)
	{
		btRigidBody* body = m_nonStaticRigidBodies[i];
		if (!body->isStaticOrKinematicObject())
		{
			//don't integrate/update velocities here, it happens in the constraint solver

			body->applyDamping(timeStep);

			body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
		}
	}
}

void exCopyDDWorld::startProfiling(btScalar timeStep)
{
	(void)timeStep;

#ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif  //BT_NO_PROFILE
}

void exCopyDDWorld::debugDrawConstraint(btTypedConstraint* constraint)
{
	bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
	btScalar dbgDrawSize = constraint->getDbgDrawSize();
	if (dbgDrawSize <= btScalar(0.f))
	{
		return;
	}

	switch (constraint->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
		{
			btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)constraint;
			btTransform tr;
			tr.setIdentity();
			btVector3 pivot = p2pC->getPivotInA();
			pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot;
			tr.setOrigin(pivot);
			getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			// that ideally should draw the same frame
			pivot = p2pC->getPivotInB();
			pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot;
			tr.setOrigin(pivot);
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
		}
		break;
		case HINGE_CONSTRAINT_TYPE:
		{
			btHingeConstraint* pHinge = (btHingeConstraint*)constraint;
			btTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			btScalar minAng = pHinge->getLowerLimit();
			btScalar maxAng = pHinge->getUpperLimit();
			if (minAng == maxAng)
			{
				break;
			}
			bool drawSect = true;
			if (!pHinge->hasLimit())
			{
				minAng = btScalar(0.f);
				maxAng = SIMD_2_PI;
				drawSect = false;
			}
			if (drawLimits)
			{
				btVector3& center = tr.getOrigin();
				btVector3 normal = tr.getBasis().getColumn(2);
				btVector3 axis = tr.getBasis().getColumn(0);
				getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btVector3(0, 0, 0), drawSect);
			}
		}
		break;
		case CONETWIST_CONSTRAINT_TYPE:
		{
			btConeTwistConstraint* pCT = (btConeTwistConstraint*)constraint;
			btTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			if (drawLimits)
			{
				//const btScalar length = btScalar(5);
				const btScalar length = dbgDrawSize;
				static int nSegments = 8 * 4;
				btScalar fAngleInRadians = btScalar(2. * 3.1415926) * (btScalar)(nSegments - 1) / btScalar(nSegments);
				btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
				pPrev = tr * pPrev;
				for (int i = 0; i < nSegments; i++)
				{
					fAngleInRadians = btScalar(2. * 3.1415926) * (btScalar)i / btScalar(nSegments);
					btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
					pCur = tr * pCur;
					getDebugDrawer()->drawLine(pPrev, pCur, btVector3(0, 0, 0));

					if (i % (nSegments / 8) == 0)
						getDebugDrawer()->drawLine(tr.getOrigin(), pCur, btVector3(0, 0, 0));

					pPrev = pCur;
				}
				btScalar tws = pCT->getTwistSpan();
				btScalar twa = pCT->getTwistAngle();
				bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btScalar(0.f));
				if (useFrameB)
				{
					tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
				}
				else
				{
					tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
				}
				btVector3 pivot = tr.getOrigin();
				btVector3 normal = tr.getBasis().getColumn(0);
				btVector3 axis1 = tr.getBasis().getColumn(1);
				getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa - tws, -twa + tws, btVector3(0, 0, 0), true);
			}
		}
		break;
		case D6_SPRING_CONSTRAINT_TYPE:
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
			btTransform tr = p6DOF->getCalculatedTransformA();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = p6DOF->getCalculatedTransformB();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			if (drawLimits)
			{
				tr = p6DOF->getCalculatedTransformA();
				const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
				btVector3 up = tr.getBasis().getColumn(2);
				btVector3 axis = tr.getBasis().getColumn(0);
				btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
				btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
				btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
				btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
				getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0, 0, 0));
				axis = tr.getBasis().getColumn(1);
				btScalar ay = p6DOF->getAngle(1);
				btScalar az = p6DOF->getAngle(2);
				btScalar cy = btCos(ay);
				btScalar sy = btSin(ay);
				btScalar cz = btCos(az);
				btScalar sz = btSin(az);
				btVector3 ref;
				ref[0] = cy * cz * axis[0] + cy * sz * axis[1] - sy * axis[2];
				ref[1] = -sz * axis[0] + cz * axis[1];
				ref[2] = cz * sy * axis[0] + sz * sy * axis[1] + cy * axis[2];
				tr = p6DOF->getCalculatedTransformB();
				btVector3 normal = -tr.getBasis().getColumn(0);
				btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
				btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
				if (minFi > maxFi)
				{
					getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0, 0, 0), false);
				}
				else if (minFi < maxFi)
				{
					getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0, 0, 0), true);
				}
				tr = p6DOF->getCalculatedTransformA();
				btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
				btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
				getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0, 0, 0));
			}
		}
		break;
		///note: the code for D6_SPRING_2_CONSTRAINT_TYPE is identical to D6_CONSTRAINT_TYPE, the D6_CONSTRAINT_TYPE+D6_SPRING_CONSTRAINT_TYPE will likely become obsolete/deprecated at some stage
		case D6_SPRING_2_CONSTRAINT_TYPE:
		{
			{
				btGeneric6DofSpring2Constraint* p6DOF = (btGeneric6DofSpring2Constraint*)constraint;
				btTransform tr = p6DOF->getCalculatedTransformA();
				if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if (drawLimits)
				{
					tr = p6DOF->getCalculatedTransformA();
					const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btVector3 up = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					if (minTh <= maxTh)
					{
						btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
						btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
						getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0, 0, 0));
					}
					axis = tr.getBasis().getColumn(1);
					btScalar ay = p6DOF->getAngle(1);
					btScalar az = p6DOF->getAngle(2);
					btScalar cy = btCos(ay);
					btScalar sy = btSin(ay);
					btScalar cz = btCos(az);
					btScalar sz = btSin(az);
					btVector3 ref;
					ref[0] = cy * cz * axis[0] + cy * sz * axis[1] - sy * axis[2];
					ref[1] = -sz * axis[0] + cz * axis[1];
					ref[2] = cz * sy * axis[0] + sz * sy * axis[1] + cy * axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn(0);
					btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if (minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0, 0, 0), false);
					}
					else if (minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0, 0, 0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0, 0, 0));
				}
			}
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
			btTransform tr = pSlider->getCalculatedTransformA();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			tr = pSlider->getCalculatedTransformB();
			if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			if (drawLimits)
			{
				btTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
				btVector3 li_min = tr * btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
				btVector3 li_max = tr * btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
				getDebugDrawer()->drawLine(li_min, li_max, btVector3(0, 0, 0));
				btVector3 normal = tr.getBasis().getColumn(0);
				btVector3 axis = tr.getBasis().getColumn(1);
				btScalar a_min = pSlider->getLowerAngLimit();
				btScalar a_max = pSlider->getUpperAngLimit();
				const btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
				getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0, 0, 0), true);
			}
		}
		break;
		default:
			break;
	}
	return;
}

void exCopyDDWorld::setConstraintSolver(btConstraintSolver* solver)
{
	if (m_ownsConstraintSolver)
	{
		btAlignedFree(m_constraintSolver);
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
	m_solverIslandCallback->m_solver = solver;
}

btConstraintSolver* exCopyDDWorld::getConstraintSolver()
{
	return m_constraintSolver;
}

int exCopyDDWorld::getNumConstraints() const
{
	return int(m_constraints.size());
}
btTypedConstraint* exCopyDDWorld::getConstraint(int index)
{
	return m_constraints[index];
}
const btTypedConstraint* exCopyDDWorld::getConstraint(int index) const
{
	return m_constraints[index];
}

void exCopyDDWorld::serializeRigidBodies(btSerializer* serializer)
{
	int i;
	//serialize all collision objects
	for (i = 0; i < m_collisionObjects.size(); i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() & btCollisionObject::CO_RIGID_BODY)
		{
			int len = colObj->calculateSerializeBufferSize();
			btChunk* chunk = serializer->allocate(len, 1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk, structType, BT_RIGIDBODY_CODE, colObj);
		}
	}

	for (i = 0; i < m_constraints.size(); i++)
	{
		btTypedConstraint* constraint = m_constraints[i];
		int size = constraint->calculateSerializeBufferSize();
		btChunk* chunk = serializer->allocate(size, 1);
		const char* structType = constraint->serialize(chunk->m_oldPtr, serializer);
		serializer->finalizeChunk(chunk, structType, BT_CONSTRAINT_CODE, constraint);
	}
}

void exCopyDDWorld::serializeDynamicsWorldInfo(btSerializer* serializer)
{
#ifdef BT_USE_DOUBLE_PRECISION
	int len = sizeof(btDynamicsWorldDoubleData);
	btChunk* chunk = serializer->allocate(len, 1);
	btDynamicsWorldDoubleData* worldInfo = (btDynamicsWorldDoubleData*)chunk->m_oldPtr;
#else   //BT_USE_DOUBLE_PRECISION
	int len = sizeof(btDynamicsWorldFloatData);
	btChunk* chunk = serializer->allocate(len, 1);
	btDynamicsWorldFloatData* worldInfo = (btDynamicsWorldFloatData*)chunk->m_oldPtr;
#endif  //BT_USE_DOUBLE_PRECISION

	memset(worldInfo, 0x00, len);

	m_gravity.serialize(worldInfo->m_gravity);
	worldInfo->m_solverInfo.m_tau = getSolverInfo().m_tau;
	worldInfo->m_solverInfo.m_damping = getSolverInfo().m_damping;
	worldInfo->m_solverInfo.m_friction = getSolverInfo().m_friction;
	worldInfo->m_solverInfo.m_timeStep = getSolverInfo().m_timeStep;

	worldInfo->m_solverInfo.m_restitution = getSolverInfo().m_restitution;
	worldInfo->m_solverInfo.m_maxErrorReduction = getSolverInfo().m_maxErrorReduction;
	worldInfo->m_solverInfo.m_sor = getSolverInfo().m_sor;
	worldInfo->m_solverInfo.m_erp = getSolverInfo().m_erp;

	worldInfo->m_solverInfo.m_erp2 = getSolverInfo().m_erp2;
	worldInfo->m_solverInfo.m_globalCfm = getSolverInfo().m_globalCfm;
	worldInfo->m_solverInfo.m_splitImpulsePenetrationThreshold = getSolverInfo().m_splitImpulsePenetrationThreshold;
	worldInfo->m_solverInfo.m_splitImpulseTurnErp = getSolverInfo().m_splitImpulseTurnErp;

	worldInfo->m_solverInfo.m_linearSlop = getSolverInfo().m_linearSlop;
	worldInfo->m_solverInfo.m_warmstartingFactor = getSolverInfo().m_warmstartingFactor;
	worldInfo->m_solverInfo.m_maxGyroscopicForce = getSolverInfo().m_maxGyroscopicForce;
	worldInfo->m_solverInfo.m_singleAxisRollingFrictionThreshold = getSolverInfo().m_singleAxisRollingFrictionThreshold;

	worldInfo->m_solverInfo.m_numIterations = getSolverInfo().m_numIterations;
	worldInfo->m_solverInfo.m_solverMode = getSolverInfo().m_solverMode;
	worldInfo->m_solverInfo.m_restingContactRestitutionThreshold = getSolverInfo().m_restingContactRestitutionThreshold;
	worldInfo->m_solverInfo.m_minimumSolverBatchSize = getSolverInfo().m_minimumSolverBatchSize;

	worldInfo->m_solverInfo.m_splitImpulse = getSolverInfo().m_splitImpulse;

	
#ifdef BT_USE_DOUBLE_PRECISION
	const char* structType = "btDynamicsWorldDoubleData";
#else   //BT_USE_DOUBLE_PRECISION
	const char* structType = "btDynamicsWorldFloatData";
#endif  //BT_USE_DOUBLE_PRECISION
	serializer->finalizeChunk(chunk, structType, BT_DYNAMICSWORLD_CODE, worldInfo);
}

void exCopyDDWorld::serialize(btSerializer* serializer)
{
	serializer->startSerialization();

	serializeDynamicsWorldInfo(serializer);

	serializeCollisionObjects(serializer);

	serializeRigidBodies(serializer);

	serializeContactManifolds(serializer);

	serializer->finishSerialization();
}
void exCopyDDWorld::getManifoldDataToDraw()
{


	for (int i = 0; i < this->m_predictiveManifolds.size(); i++)
	{
		btPersistentManifold* current = m_predictiveManifolds[i];
		int num_points = current->getNumContacts();
		for (int j = 0; j < num_points; j++)
		{
			btManifoldPoint pt = current->getContactPoint(j);
			TmpDataStorage->updateData(pt.m_appliedImpulse);
			TmpDataStorage->updateData(pt.m_appliedImpulseLateral1);
			TmpDataStorage->updateData(pt.m_appliedImpulseLateral2);
			TmpDataStorage->updateData(pt.m_combinedContactDamping1);
			TmpDataStorage->updateData(pt.m_combinedContactStiffness1);
			TmpDataStorage->updateData(pt.m_combinedFriction);
			TmpDataStorage->updateData(pt.m_combinedRestitution);
			TmpDataStorage->updateData(pt.m_contactCFM);
			TmpDataStorage->updateData(pt.m_contactERP);
			TmpDataStorage->updateData(pt.m_contactMotion1);
			TmpDataStorage->updateData(pt.m_contactMotion2);
			TmpDataStorage->updateData(pt.m_contactPointFlags);
			TmpDataStorage->updateData(pt.m_distance1);
			TmpDataStorage->updateData(pt.m_frictionCFM);
			TmpDataStorage->updateData(pt.m_index0);
			TmpDataStorage->updateData(pt.m_index1);
			TmpDataStorage->updateData(pt.m_lateralFrictionDir1.getX());
			TmpDataStorage->updateData(pt.m_lateralFrictionDir1.getY());
			TmpDataStorage->updateData(pt.m_lateralFrictionDir1.getZ());

			TmpDataStorage->updateData(pt.m_lateralFrictionDir2.getX());
			TmpDataStorage->updateData(pt.m_lateralFrictionDir2.getY());
			TmpDataStorage->updateData(pt.m_lateralFrictionDir2.getZ());
			TmpDataStorage->updateData(pt.m_lifeTime);
			TmpDataStorage->updateData(pt.m_localPointA.getX());
			TmpDataStorage->updateData(pt.m_localPointA.getY());
			TmpDataStorage->updateData(pt.m_localPointA.getZ());

			TmpDataStorage->updateData(pt.m_localPointB.getX());
			TmpDataStorage->updateData(pt.m_localPointB.getY());
			TmpDataStorage->updateData(pt.m_localPointB.getZ());

			TmpDataStorage->updateData(pt.m_normalWorldOnB.getX());
			TmpDataStorage->updateData(pt.m_normalWorldOnB.getY());
			TmpDataStorage->updateData(pt.m_normalWorldOnB.getZ());

			TmpDataStorage->updateData(pt.m_partId0);
			TmpDataStorage->updateData(pt.m_partId1);
			TmpDataStorage->updateData(pt.m_positionWorldOnA.getX());
			TmpDataStorage->updateData(pt.m_positionWorldOnA.getY());
			TmpDataStorage->updateData(pt.m_positionWorldOnA.getZ());

			TmpDataStorage->updateData(pt.m_positionWorldOnB.getX());
			TmpDataStorage->updateData(pt.m_positionWorldOnB.getY());
			TmpDataStorage->updateData(pt.m_positionWorldOnB.getZ());

			TmpDataStorage->updateData(pt.m_prevRHS);
		}
	}
}
exConstSolvDataStorage* exCopyDDWorld::getTmpSolverData()
{
	return m_constraintSolver->TmpDataStorage;
}
exConstSolvDataStorage* exCopyDDWorld::getManifoldData()
{
	return TmpDataStorage;
}
void exCopyDDWorld::resetAllDrawData()
{
	m_constraintSolver->TmpDataStorage->reset();
	TmpDataStorage->reset();
}


void exCopyDDWorld::clearIslandCallbackData()
{
	m_solverIslandCallback->m_island_data.m_savedpoint.resizeNoInitialize(0);
}
exCopyDDWorld::IslandData* exCopyDDWorld::getIslandCallbackDataPt()
{
	return &m_solverIslandCallback->m_island_data;
}

btConstraintArray* exCopyDDWorld::getTmpSeqImplStorage()
{
	btSequentialImpulseConstraintSolver* trg = (btSequentialImpulseConstraintSolver*)m_constraintSolver;
	return &trg->m_tmp_storage;
}

void exCopyDDWorld::clearTmpSeqImplStorage()
{
	btSequentialImpulseConstraintSolver* trg = (btSequentialImpulseConstraintSolver*)m_constraintSolver;
	trg->m_tmp_storage.clear();
}

int exCopyDDWorld::stepSimulationPart0(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep)
{
	printf("stepSimulation\n");
	startProfiling(timeStep);

	int numSimulationSubSteps = 0;

	if (maxSubSteps)
	{
		//fixed timestep with interpolation
		m_fixedTimeStep = fixedTimeStep;
		m_localTime += timeStep;
		if (m_localTime >= fixedTimeStep)
		{
			numSimulationSubSteps = int(m_localTime / fixedTimeStep);
			m_localTime -= numSimulationSubSteps * fixedTimeStep;
		}
	}
	else
	{
		//variable timestep
		fixedTimeStep = timeStep;
		m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
		m_fixedTimeStep = 0;
		if (btFuzzyZero(timeStep))
		{
			numSimulationSubSteps = 0;
			maxSubSteps = 0;
		}
		else
		{
			numSimulationSubSteps = 1;
			maxSubSteps = 1;
		}
	}

	//process some debugging flags
	if (getDebugDrawer())
	{
		btIDebugDraw* debugDrawer = getDebugDrawer();
		gDisableDeactivation = (debugDrawer->getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0;
	}
	return numSimulationSubSteps;
}


int exCopyDDWorld::stepSimulationPart1(btScalar fixedTimeStep, int clampedSimulationSteps)
{
	saveKinematicState(fixedTimeStep * clampedSimulationSteps);

	applyGravity();

	printf("clamped simulation steps: %d\n", clampedSimulationSteps);
	return 0;
}
void exCopyDDWorld::stepSimulationPart2(btScalar timeStep)
{
	BT_PROFILE("internalSingleStepSimulation");

	if (0 != m_internalPreTickCallback)
	{
		(*m_internalPreTickCallback)(this, timeStep);
	}

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);
}
void exCopyDDWorld::stepSimulationPart3(btScalar timeStep)
{
	btDispatcherInfo& dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	createPredictiveContacts(timeStep);
}
void exCopyDDWorld::stepSimulationPart4()
{
	btDbvtBroadphase* brph = static_cast<btDbvtBroadphase*>(m_broadphasePairCache);
	btHashedOverlappingPairCache* hashed = static_cast<btHashedOverlappingPairCache*>(brph->m_paircache);
	///perform collision detection
	printf("Get num manifolds 4: %d\n", getDispatcher()->getNumManifolds());
	performDiscreteCollisionDetection();
	printf("hashed: %d\n", hashed->GetCount());

	printf("Get num manifolds 4: %d\n", getDispatcher()->getNumManifolds());
}
	//getManifoldDataToDraw(); //#Debug
void exCopyDDWorld::stepSimulationPart5()
{
	calculateSimulationIslands();
}
void exCopyDDWorld::stepSimulationPart6(btScalar timeStep)
{
	getSolverInfo().m_timeStep = timeStep;

	///solve contact and other joint constraints
	solveConstraints(getSolverInfo());
}
void exCopyDDWorld::stepSimulationPart7(btScalar timeStep)
{
	///CallbackTriggers();

	///integrate transforms

	integrateTransforms(timeStep);
}
void exCopyDDWorld::stepSimulationPart8(btScalar timeStep)
{
	///update vehicle simulation
	updateActions(timeStep);

	updateActivationState(timeStep);
}
void exCopyDDWorld::stepSimulationPart9(btScalar timeStep)
{
	if (0 != m_internalTickCallback)
	{
		(*m_internalTickCallback)(this, timeStep);
	}

	synchronizeMotionStates();
}


void exCopyDDWorld::stepSimulationPart10()
{
	synchronizeMotionStates();
}
void exCopyDDWorld::stepSimulationPart11()
{
	clearForces();
}


