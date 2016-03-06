/*
cimulator plugin for SanAndreas Multiplayer
Copyright (c) 2016 codectile

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CCOLLISION_H
#define CCOLLISION_H

struct CustomCollisionSensor : public btCollisionWorld::ContactResultCallback
{
	int hit;
	btVector3 contactPointA, contactPointB;
	btScalar penetrationDepth;
	CustomCollisionSensor() : btCollisionWorld::ContactResultCallback(), hit(0), contactPointA(0, 0, 0), contactPointB(0, 0, 0)
	{
	}

	virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0, int partId0, int index0, const btCollisionObjectWrapper* colObj1, int partId1, int index1)
	{
		hit = 1;
		contactPointA = cp.getPositionWorldOnA();
		contactPointB = cp.getPositionWorldOnB();
		penetrationDepth = cp.getDistance();
		printf("\ncolObj0: %i", colObj0->m_collisionObject->getUserIndex());
		printf("\ncolObj1: %i", colObj1->m_collisionObject->getUserIndex());
		return 0;
	}
};

int cr_readCadb();
int cr_readCCF();
void cr_createColVehicle();
void cr_createColObject();
void cr_removeVehicleCol(btDiscreteDynamicsWorld* dynamicsWorld, btCollisionObject* col);
btCollisionObject* cr_createVehicleCollision(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btVector3& position, btQuaternion& rotation);
void cr_objectPlacement(btDiscreteDynamicsWorld* dynamicsWorld, btScalar worldrest);
void cr_removeBuilding(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btScalar x, btScalar y, btScalar z, btScalar radius);
btCollisionObject* cr_addStaticCollision(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btVector3& position, btVector3& rotatation);
btRigidBody* cr_addDynamicCollision(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btScalar mass, btVector3& position, btVector3& rotation, int state);
void cr_removeDynamicCol(btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* rigidBody);
void cr_removeStaticCol(btDiscreteDynamicsWorld* dynamicsWorld, btCollisionObject* colObj);
void cr_deleteColBody(btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* rigidBody);
void cr_setCollisionShape(btCollisionObject* rigidBody, int modelid);
void cr_getBoundingSphere(int modelid, btVector3& center, btScalar &radius);
void cr_getAABB(int modelid, btVector3& min, btVector3& max);
int cr_isCompound(int modelid);
int cr_getNumChildShapes(int modelid);
void cr_removeColMap(btDiscreteDynamicsWorld* dynamicsWorld);
void cr_removeRoadBlocks(btDiscreteDynamicsWorld* dynamicsWorld);
void cr_unLoad(btDiscreteDynamicsWorld* dynamicsWorld);
btCollisionObject* cr_createCapsuleCharacter(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& position, btScalar radius, btScalar height);
btGhostObject* cr_createGhost(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btVector3& position, btVector3& rotation);
int cr_getNumOverlappingObjects(btGhostObject* ghost);
void cr_removeGhost(btDiscreteDynamicsWorld* dynamicsWorld, btGhostObject* ghost);
void cr_deleteGhost(btDiscreteDynamicsWorld* dynamicsWorld, btGhostObject* ghost);

static btCompoundShape* static_shapes;
static btCompoundShape* dynamic_shapes;
static btCompoundShape* vehicle_shapes;
#endif // !CCOLLISION_H
