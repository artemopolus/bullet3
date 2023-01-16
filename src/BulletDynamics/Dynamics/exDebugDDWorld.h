
#ifndef EX_DEBUG_DDWORLD_H
#define EX_DEBUG_DDWORLD_H

#include "btDiscreteDynamicsWorld.h"

ATTRIBUTE_ALIGNED16(class)
exDebugDDWorld : public btDiscreteDynamicsWorld
{


public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
	exDebugDDWorld(btDispatcher * dispatcher, btBroadphaseInterface * pairCache, btConstraintSolver * constraintSolver, btCollisionConfiguration * collisionConfiguration);

	virtual ~exDebugDDWorld();

}


#endif //EX_DEBUG_DDWORLD_H
