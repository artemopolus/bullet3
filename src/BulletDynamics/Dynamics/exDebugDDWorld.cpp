#include "exDebugDDWorld.h"

exDebugDDWorld::exDebugDDWorld(btDispatcher * dispatcher, btBroadphaseInterface * pairCache, btConstraintSolver * constraintSolver, btCollisionConfiguration * collisionConfiguration)
	: btDiscreteDynamicsWorld( dispatcher,  pairCache,  constraintSolver,  collisionConfiguration)
{

}
exDebugDDWorld::~exDebugDDWorld()
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
