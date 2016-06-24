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

#define SIMD_DEG_TO_RAD	 btScalar(0.01745329251994329576923690768489)
#define SIMD_RADIAN_TO_DEG	 btScalar(57.295779513082320876798154814105)

//custom collision sensor, it senses whether an objects is in contact with other or not
struct CustomCollisionSensor : public btCollisionWorld::ContactResultCallback
{
	int hits;
	btVector3 contactPointA, contactPointB;
	btScalar penetrationDepth;
	CustomCollisionSensor() : btCollisionWorld::ContactResultCallback(), hits(0), contactPointA(0, 0, 0), contactPointB(0, 0, 0)
	{
	}

	virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0, int partId0, int index0, const btCollisionObjectWrapper* colObj1, int partId1, int index1)
	{
		hits++; //determines that how many objects were in contact with the query object
		contactPointA = cp.getPositionWorldOnA();
		contactPointB = cp.getPositionWorldOnB();
		penetrationDepth = cp.getDistance();
		//printf("\ncolObj0: %i", colObj0->m_collisionObject->getUserIndex());
		//printf("\ncolObj1: %i", colObj1->m_collisionObject->getUserIndex());
		return 0;
	}
};

// simple user data structure
struct UserData
{
	int m_integerType;
	float m_floatType;
	UserData() : m_integerType(0), m_floatType(0.f)
	{

	}
	~UserData()
	{
		m_integerType = 0;
		m_floatType = 0.f;
	}
};

int cr_readCadb();
int cr_readCCF();
void cr_createColVehicle();
void cr_createColObject();
btCollisionObject* cr_createVehicleCollision(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btVector3& position, btQuaternion& rotation);
void cr_objectPlacement(btDiscreteDynamicsWorld* dynamicsWorld, btScalar worldrest);
void cr_loadWater(btDiscreteDynamicsWorld* dynamicsWorld);
void cr_removeBuilding(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btScalar x, btScalar y, btScalar z, btScalar radius);
btCollisionObject* cr_addStaticCollision(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btVector3& position, btVector3& rotatation);
btRigidBody* cr_addDynamicCollision(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btScalar mass, btVector3& position, btVector3& rotation, int inertia, int state);
void cr_removeDynamicCol(btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* rigidBody);
void cr_removeStaticCol(btDiscreteDynamicsWorld* dynamicsWorld, btCollisionObject* colObj);
void cr_deleteColBody(btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* rigidBody);
void cr_setCollisionShape(btCollisionObject* rigidBody, int modelid);
void cr_getBoundingSphere(int modelid, btVector3& center, btScalar &radius);
void cr_getAABB(int modelid, btVector3& pos, btVector3& rot, btVector3& min, btVector3& max);
int cr_isCompound(int modelid);
int cr_getNumChildShapes(int modelid);
void cr_removeColMap(btDiscreteDynamicsWorld* dynamicsWorld);
void cr_removeRoadBlocks(btDiscreteDynamicsWorld* dynamicsWorld);
void cr_unLoad(btDiscreteDynamicsWorld* dynamicsWorld);
btCollisionObject* cr_createCapsuleCharacter(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& position, btScalar radius, btScalar height);

extern btCompoundShape* static_shapes;
extern btCompoundShape* dynamic_shapes;
extern btCompoundShape* vehicle_shapes_st;
extern btCompoundShape* vehicle_shapes_dn;

extern uint16_t vehicleRef[612];
#endif // !CCOLLISION_H
