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

#include "CNatives.h"

// enables simulation
cell AMX_NATIVE_CALL CR_EnableSimulation(AMX* amx, cell* params)
{
	simFlag = 1;
	Newtime = Oldtime = GetTickCount64();
	return 1;
}

// disables simulation
cell AMX_NATIVE_CALL CR_DisableSimulation(AMX* amx, cell* params)
{
	simFlag = 0;
	return 1;
}

// loads the collision map
cell AMX_NATIVE_CALL CR_Load(AMX* amx, cell* params)
{
	unsigned long time = clock->getTimeMilliseconds();
	cr_objectPlacement(dynamicsWorld, amx_ctof(params[1]));
	logprintf("Time taken to load the collision map: %ims", clock->getTimeMilliseconds() - time);
	cr_removeRoadBlocks(dynamicsWorld);
	return 1;
}

// removes the collision map
cell AMX_NATIVE_CALL CR_RemoveColMap(AMX* amx, cell* params)
{
	cr_unLoad(dynamicsWorld);
	return 1;
}

// removes building for a given range
cell AMX_NATIVE_CALL CR_RemoveBuilding(AMX* amx, cell* params)
{
	cr_removeBuilding(dynamicsWorld, params[1], amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]), amx_ctof(params[5]));
	return 1;
}

// sets the world gravity
cell AMX_NATIVE_CALL CR_SetWorldGravity(AMX* amx, cell* params)
{
	cr_setWorldGravity(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])));
	return 1;
}

// returns world gravity
cell AMX_NATIVE_CALL CR_GetWorldGravity(AMX* amx, cell* params)
{
	btVector3 gravity = cr_getWorldGravity(dynamicsWorld);
	cell* addr[3];
	amx_GetAddr(amx, params[1], &addr[0]);
	amx_GetAddr(amx, params[2], &addr[1]);
	amx_GetAddr(amx, params[3], &addr[2]);
	*addr[0] = amx_ftoc(gravity.getX());
	*addr[1] = amx_ftoc(gravity.getY());
	*addr[2] = amx_ftoc(gravity.getZ());
	return 1;
}

// simple ray trace, hit point and normals are passed by reference
cell AMX_NATIVE_CALL CR_RayCastNormal(AMX* amx, cell* params)
{
	btVector3 normal;
	if (cr_rayCastNormal(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), normal))
	{
		cell* addr[3];
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		*addr[0] = amx_ftoc(normal.getX());
		*addr[1] = amx_ftoc(normal.getY());
		*addr[2] = amx_ftoc(normal.getZ());
		return 1;
	}
	return 0;
}

// simple ray tracing, hit points are passed by reference
cell AMX_NATIVE_CALL CR_RayCast(AMX* amx, cell* params)
{
	btVector3 normal;
	if (cr_rayCast(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), normal))
	{
		cell* addr[3];
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		*addr[0] = amx_ftoc(normal.getX());
		*addr[1] = amx_ftoc(normal.getY());
		*addr[2] = amx_ftoc(normal.getZ());
		return 1;
	}
	return 0;
}

// traces an invisible ray, modelid is passed by reference
cell AMX_NATIVE_CALL CR_RayCastEx(AMX* amx, cell* params)
{
	btVector3 hitpoint;
	int modelid;
	if (cr_rayCastEx(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), hitpoint, modelid))
	{
		cell* addr[3];
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		amx_GetAddr(amx, params[10], &addr[3]);
		*addr[0] = amx_ftoc(hitpoint.getX());
		*addr[1] = amx_ftoc(hitpoint.getY());
		*addr[2] = amx_ftoc(hitpoint.getZ());
		*addr[3] = modelid;
		return 1;
	}
	return 0;
}

// angle and reflection vector is passed by reference
cell AMX_NATIVE_CALL CR_RayCastReflection(AMX* amx, cell* params)
{
	btVector3 reflection;
	btScalar angle;
	if (cr_rayCastReflection(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), reflection, angle))
	{
		cell* addr[3];
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		amx_GetAddr(amx, params[10], &addr[3]);
		*addr[0] = amx_ftoc(reflection.getX());
		*addr[1] = amx_ftoc(reflection.getY());
		*addr[2] = amx_ftoc(reflection.getZ());
		*addr[3] = amx_ftoc(angle);
		return 1;
	}
	return 0;
}

// passes object's information which got hit by a virtual ray
cell AMX_NATIVE_CALL CR_RayCastObjectInfo(AMX* amx, cell* params)
{
	int modelid, isStatic;
	float rad;
	if (cr_rayCastInfo(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), modelid, rad, isStatic))
	{
		cell* addr[3];
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		*addr[0] = modelid;
		*addr[1] = amx_ftoc(rad);
		*addr[2] = isStatic;
		return 1;
	}
	return 0;
}

cell AMX_NATIVE_CALL CR_RayCastIntData(AMX* amx, cell* params)
{
	btVector3 hitpoint;
	void *pData = NULL;
	pData = cr_rayCastData(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), hitpoint);
	if (pData)
	{
		cell* addr[4] = { NULL };
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		amx_GetAddr(amx, params[10], &addr[3]);

		*addr[0] = amx_ftoc(hitpoint.getX());
		*addr[1] = amx_ftoc(hitpoint.getY());
		*addr[2] = amx_ftoc(hitpoint.getZ());
		*addr[3] = ((UserData*)pData)->m_integerType;
		return 1;
	}
	return 0;
}

cell AMX_NATIVE_CALL CR_RayCastFloatData(AMX* amx, cell* params)
{
	btVector3 hitpoint;
	void *pData = NULL;
	pData = cr_rayCastData(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), hitpoint);
	if (pData)
	{
		cell* addr[4] = { NULL };
		amx_GetAddr(amx, params[7], &addr[0]);
		amx_GetAddr(amx, params[8], &addr[1]);
		amx_GetAddr(amx, params[9], &addr[2]);
		amx_GetAddr(amx, params[10], &addr[3]);

		*addr[0] = amx_ftoc(hitpoint.getX());
		*addr[1] = amx_ftoc(hitpoint.getY());
		*addr[2] = amx_ftoc(hitpoint.getZ());
		*addr[3] = amx_ftoc(((UserData*)pData)->m_floatType);
		return 1;
	}
	return 0;
}

// creates dynamic collision volume
cell AMX_NATIVE_CALL CR_CreateDynamicCol(AMX* amx, cell* params)
{
	/*if (!index_availability_dynamic[params[1]])
	{
		SAFE_DELETE(rigidBody[params[1]]);
		index_availability_dynamic[params[1]] = 1;
	}*/
	int index = GetEmptyDynamicIndex();
	if (index == -1)
	{
		logprintf("cimulator::maximum dynamic object reached");
		return -1;
	}
	rigidBody[index] = new DynamicObject(index, params[2], params[3], cr_addDynamicCollision(dynamicsWorld, params[3], amx_ctof(params[4]), btVector3(amx_ctof(params[5]), amx_ctof(params[6]), amx_ctof(params[7])), btVector3(amx_ctof(params[8]), amx_ctof(params[9]), amx_ctof(params[10])), params[11], params[12]));
	rigidBody[index]->col->setUserIndex(params[3]); //set the modelid
	index_availability_dynamic[index] = 0; //make it unavailable
	return index;
}

// creates static collision volume
cell AMX_NATIVE_CALL CR_CreateStaticCol(AMX* amx, cell* params)
{
	/*if (!index_availability_static[params[1]])
	{
		SAFE_DELETE(staticBody[params[1]]);
		index_availability_static[params[1]] = 1;
	}*/
	int index = GetEmptyStaticIndex();
	if (index == -1)
	{
		logprintf("cimulator::maximum static object reached");
		return -1;
	}
	staticBody[index] = new StaticObject(index, params[2], params[3], cr_addStaticCollision(dynamicsWorld, params[3], btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), btVector3(amx_ctof(params[7]), amx_ctof(params[8]), amx_ctof(params[9]))));
	staticBody[index]->col->setUserIndex(params[3]); //set the modelid
	index_availability_static[index] = 0; //make it unavailable
	return 1;
}

// removes the dynamic collision volume from the world
cell AMX_NATIVE_CALL CR_RemoveDynamicCol(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		cr_removeDynamicCol(dynamicsWorld, rigidBody[params[1]]->col); //remove the body from the world
		index_availability_dynamic[params[1]] = 1; //make it available
		SAFE_DELETE(rigidBody[params[1]]);
		SLEEP(15); //wait...
	}
	return 1;
}

// removes the static collision volume from the world
cell AMX_NATIVE_CALL CR_RemoveStaticCol(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		cr_removeStaticCol(dynamicsWorld, staticBody[params[1]]->col); //remove the body from the world
		index_availability_static[params[1]] = 1; //make it available
		SAFE_DELETE(staticBody[params[1]]);
		SLEEP(15); //wait...
	}
	return 1;
}

// get dynamic modelid
cell AMX_NATIVE_CALL CR_GetDynamicColModel(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], -1);
	if (!index_availability_dynamic[params[1]])
		return rigidBody[params[1]]->modelid; //return the modelid
	return -1;
}

// get static modelid
cell AMX_NATIVE_CALL CR_GetStaticColModel(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], -1);
	if (!index_availability_static[params[1]])
		return staticBody[params[1]]->modelid; //return the modelid
	return -1;
}

// get dynamic object id
cell AMX_NATIVE_CALL CR_GetDynamicColObject(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], -1);
	if (!index_availability_dynamic[params[1]])
		return rigidBody[params[1]]->objectid; //return the objectid
	return -1;
}

// get static object id
cell AMX_NATIVE_CALL CR_GetStaticColObject(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], -1);
	if (!index_availability_static[params[1]])
		return staticBody[params[1]]->objectid; //return the modelid
	return -1;
}

// sets mass
cell AMX_NATIVE_CALL CR_SetMass(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setMass(rigidBody[params[1]]->col, amx_ctof(params[2])); //set the mass
	return 1;
}

// returns mass
cell AMX_NATIVE_CALL CR_GetMass(AMX* amx, cell* params)
{
	btScalar mass = -1.f;
	INVALIDITY_CHECK(params[1], amx_ftoc(mass));
	if (!index_availability_dynamic[params[1]])
		mass = cr_getMass(rigidBody[params[1]]->col); //returns the mass
	return amx_ftoc(mass);
}

// sets a dynamic collision shape (real-time)
cell AMX_NATIVE_CALL CR_SetDynamicColShape(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setCollisionShape(rigidBody[params[1]]->col, params[2]); //replaces the collision shape at real-time
	SLEEP(5);
	return 1;
}

// sets a static collision shape (real-time)
cell AMX_NATIVE_CALL CR_SetStaticColShape(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
		cr_setCollisionShape(staticBody[params[1]]->col, params[2]); //replaces the collision shape at real-time
	SLEEP(5);
	return 1;
}

// returns bounding sphere
cell AMX_NATIVE_CALL CR_GetBoundingSphere(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	btVector3 center;
	btScalar radius;
	cr_getBoundingSphere(params[1], center, radius); //provides the offsets and radius
	cell* addr[4];
	amx_GetAddr(amx, params[2], &addr[0]);
	amx_GetAddr(amx, params[3], &addr[1]);
	amx_GetAddr(amx, params[4], &addr[2]);
	amx_GetAddr(amx, params[5], &addr[3]);
	*addr[0] = amx_ftoc(center.getX());
	*addr[1] = amx_ftoc(center.getY());
	*addr[2] = amx_ftoc(center.getZ());
	*addr[3] = amx_ftoc(radius);
	return 1;
}

// returns axis-aligned bounding box
cell AMX_NATIVE_CALL CR_GetAABB(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	btVector3 aabbmin, aabbmax;
	cr_getAABB(params[1], aabbmin, aabbmax); //provides the max and min vertices
	cell* addr[6];
	amx_GetAddr(amx, params[2], &addr[0]);
	amx_GetAddr(amx, params[3], &addr[1]);
	amx_GetAddr(amx, params[4], &addr[2]);
	amx_GetAddr(amx, params[5], &addr[3]);
	amx_GetAddr(amx, params[6], &addr[4]);
	amx_GetAddr(amx, params[7], &addr[5]);
	*addr[0] = amx_ftoc(aabbmin.getX());
	*addr[1] = amx_ftoc(aabbmin.getY());
	*addr[2] = amx_ftoc(aabbmin.getZ());
	*addr[3] = amx_ftoc(aabbmax.getX());
	*addr[4] = amx_ftoc(aabbmax.getY());
	*addr[5] = amx_ftoc(aabbmax.getZ());
	return 1;
}

// returns the transform
cell AMX_NATIVE_CALL CR_GetTransform(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		btVector3 pos, rot;
		cr_getColTransform(rigidBody[params[1]]->col, pos, rot); //provides the world transform
		cell* addr[6];
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		amx_GetAddr(amx, params[5], &addr[3]);
		amx_GetAddr(amx, params[6], &addr[4]);
		amx_GetAddr(amx, params[7], &addr[5]);
		*addr[0] = amx_ftoc(pos.getX());
		*addr[1] = amx_ftoc(pos.getY());
		*addr[2] = amx_ftoc(pos.getZ());
		*addr[3] = amx_ftoc(rot.getX());
		*addr[4] = amx_ftoc(rot.getY());
		*addr[5] = amx_ftoc(rot.getZ());
	}
	return 1;
}

// sets dynamic origin
cell AMX_NATIVE_CALL CR_SetDynamicOrigin(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setOrigin(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]))); //sets the origin
	return 1;
}

// sets static origin
cell AMX_NATIVE_CALL CR_SetStaticOrigin(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		btTransform transform = staticBody[params[1]]->col->getWorldTransform();
		transform.setOrigin(btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]))); //sets the origin
		staticBody[params[1]]->col->setWorldTransform(transform);
	}
	return 1;
}

// returns dynamic origin
cell AMX_NATIVE_CALL CR_GetDynamicOrigin(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		btVector3 pos;
		cell* addr[3];
		pos = cr_getOrigin(rigidBody[params[1]]->col);
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(pos.getX());
		*addr[1] = amx_ftoc(pos.getY());
		*addr[2] = amx_ftoc(pos.getZ());
	}
	return 1;
}

// returns static origin
cell AMX_NATIVE_CALL CR_GetStaticOrigin(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		btVector3 pos;
		cell* addr[3];
		pos = staticBody[params[1]]->col->getWorldTransform().getOrigin();;
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(pos.getX());
		*addr[1] = amx_ftoc(pos.getY());
		*addr[2] = amx_ftoc(pos.getZ());
	}
	return 1;
}

// sets dynamic rotations
cell AMX_NATIVE_CALL CR_SetDynamicRotation(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setRotation(rigidBody[params[1]]->col, amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]));
	return 1;
}

// sets static rotations
cell AMX_NATIVE_CALL CR_SetStaticRotation(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		btTransform transform;
		btQuaternion quat;
		quat.setEuler(amx_ctof(params[2]) * SIMD_DEG_TO_RAD, amx_ctof(params[3]) * SIMD_DEG_TO_RAD, amx_ctof(params[4]) * SIMD_DEG_TO_RAD);
		transform = staticBody[params[1]]->col->getWorldTransform();
		transform.setRotation(quat);
		staticBody[params[1]]->col->setWorldTransform(transform);
	}
	return 1;
}

// passes rotations by reference
cell AMX_NATIVE_CALL CR_GetDynamicRotation(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		btScalar rot[3];
		cell* addr[3];
		cr_getRotation(rigidBody[params[1]]->col, rot[0], rot[1], rot[2]);
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(rot[0]);
		*addr[1] = amx_ftoc(rot[1]);
		*addr[2] = amx_ftoc(rot[2]);
	}
	return 1;
}

// passes rotations by reference
cell AMX_NATIVE_CALL CR_GetStaticRotation(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		btScalar rot[3];
		cell* addr[3];
		staticBody[params[1]]->col->getWorldTransform().getBasis().getEulerYPR(rot[0], rot[1], rot[2]);
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(rot[0]);
		*addr[1] = amx_ftoc(rot[1]);
		*addr[2] = amx_ftoc(rot[2]);
	}
	return 1;
}

//sets linear velocity
cell AMX_NATIVE_CALL CR_SetLinearVelocity(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setLinearVelocity(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])));
	return 1;
}

// passes linear velocity by reference
cell AMX_NATIVE_CALL CR_GetLinearVelocity(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		btVector3 vel = cr_getLinearVelocity(rigidBody[params[1]]->col);
		cell* addr[3];
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(vel.getX());
		*addr[1] = amx_ftoc(vel.getY());
		*addr[2] = amx_ftoc(vel.getZ());
	}
	return 1;
}

// sets angular velocity
cell AMX_NATIVE_CALL CR_SetAngularVelocity(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setAngularVelocity(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])));
	return 1;
}

// passes angular velocity by reference
cell AMX_NATIVE_CALL CR_GetAngularVelocity(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		btVector3 vel = cr_getAngularVelocity(rigidBody[params[1]]->col);
		cell* addr[3];
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(vel.getX());
		*addr[1] = amx_ftoc(vel.getY());
		*addr[2] = amx_ftoc(vel.getZ());
	}
	return 1;
}

// sets torque
cell AMX_NATIVE_CALL CR_SetTorque(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setTorque(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])));
	return 1;
}

// sets friction of a collision volume
cell AMX_NATIVE_CALL CR_SetFriction(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setFriction(rigidBody[params[1]]->col, amx_ctof(params[2]));
	return 1;
}

// returns the friction of a collision volume
cell AMX_NATIVE_CALL CR_GetFriction(AMX* amx, cell* params)
{
	btScalar friction = 0.f;
	INVALIDITY_CHECK(params[1], amx_ftoc(friction));
	if (!index_availability_dynamic[params[1]])
		friction = cr_getFriction(rigidBody[params[1]]->col);
	return amx_ftoc(friction);
}

// sets the coefficient of restitution of the collision volume (avoid using it)
cell AMX_NATIVE_CALL CR_SetRestitution(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_setRestitution(rigidBody[params[1]]->col, amx_ctof(params[2]));
	return 1;
}

// returns coefficient of restitution of the collision volume
cell AMX_NATIVE_CALL CR_GetRestitution(AMX* amx, cell* params)
{
	btScalar rest = 0.f;
	INVALIDITY_CHECK(params[1], amx_ftoc(rest));
	if (!index_availability_dynamic[params[1]])
		rest = cr_getRestitution(rigidBody[params[1]]->col);
	return amx_ftoc(rest);
}

//checks whether a collision volume is active or not
cell AMX_NATIVE_CALL CR_IsActive(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		return cr_isActive(rigidBody[params[1]]->col);
	return 0;
}

// checks whether a collision volume is static or dynamic
cell AMX_NATIVE_CALL CR_IsMoving(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		btVector3 vel;
		vel = rigidBody[params[1]]->col->getLinearVelocity();
		if (vel.isZero())
			return 0;
	}
	return 1;
}

// checks if the specified slot is available for dynamic objects
cell AMX_NATIVE_CALL CR_IsDynamicSlotUsed(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	return (index_availability_dynamic[params[1]] ? 0 : 1);
}

// checks if the specified slot is available for static objects
cell AMX_NATIVE_CALL CR_IsStaticSlotUsed(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	return (index_availability_static[params[1]] ? 0 : 1);
}

// activates a deactivated collision volume
cell AMX_NATIVE_CALL CR_Activate(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
		cr_activate(rigidBody[params[1]]->col);
	return 1;
}

// simulates the dynamics world
cell AMX_NATIVE_CALL CR_Simulator(AMX* amx, cell* params)
{
	cr_simulate(dynamicsWorld, params[1], params[2]);
	return 1;
}

// returns number of collision objects created in the world (includes map)
cell AMX_NATIVE_CALL CR_GetNumCollisionObject(AMX* amx, cell* params)
{
	return cr_getNumCollisionObject(dynamicsWorld);
}

// returns number of child shapes of a collision shape
cell AMX_NATIVE_CALL CR_GetNumChildShapes(AMX* amx, cell* params)
{
	return cr_getNumChildShapes(params[1]);
}

// checks whether a collision shape is compound or not
cell AMX_NATIVE_CALL CR_IsCompound(AMX* amx, cell* params)
{
	return cr_isCompound(params[1]);
}

//pause the execution of the plugin for a given milliseconds
cell AMX_NATIVE_CALL CR_Wait(AMX* amx, cell* params)
{
	SLEEP(params[1]);
	return 1;
}

// capsule approximation
cell AMX_NATIVE_CALL CR_CharacterContactTest(AMX* amx, cell* params)
{
	CustomCollisionSensor sensor;
	btCollisionObject* ob = cr_createCapsuleCharacter(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), amx_ctof(params[4]), amx_ctof(params[5]));
	dynamicsWorld->contactTest(ob, sensor);
	SAFE_DELETE(ob);
	return sensor.hits;
}

// checks collision for a given dynamic collision volume
cell AMX_NATIVE_CALL CR_DynamicContactTest(AMX* amx, cell* params)
{
	if (!index_availability_dynamic[params[1]])
	{
		CustomCollisionSensor callback;
		dynamicsWorld->updateSingleAabb(rigidBody[params[1]]->col);
		dynamicsWorld->contactTest(rigidBody[params[1]]->col, callback);
		return callback.hits;
	}
	return 0;
}

// checks collision for a given modelid
cell AMX_NATIVE_CALL CR_ProxyContactTest(AMX* amx, cell* params)
{
	CustomCollisionSensor callback;
	btQuaternion quat;
	quat.setEulerZYX(amx_ctof(params[5]), amx_ctof(params[6]), amx_ctof(params[7]));
	btCollisionObject *body = new btCollisionObject();
	body->setCollisionShape((btCollisionShape*)&dynamic_shapes[params[1]]);
	body->setCollisionFlags(1);
	body->setWorldTransform(btTransform(quat, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]))));
	dynamicsWorld->contactTest(body, callback);
	delete body;
	SLEEP(1);
	return callback.hits;
}

// point of contact of two objects are passed by reference
cell AMX_NATIVE_CALL CR_GetContactPoints(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], -1);
	if (!index_availability_dynamic[params[1]])
	{
		CustomCollisionSensor callback;
		dynamicsWorld->contactTest(rigidBody[params[1]]->col, callback);
		if (callback.hits)
		{
			cell* addr[6];
			amx_GetAddr(amx, params[2], &addr[0]);
			amx_GetAddr(amx, params[3], &addr[1]);
			amx_GetAddr(amx, params[4], &addr[2]);
			amx_GetAddr(amx, params[5], &addr[3]);
			amx_GetAddr(amx, params[6], &addr[4]);
			amx_GetAddr(amx, params[7], &addr[5]);
			*addr[0] = amx_ftoc(callback.contactPointA.getX());
			*addr[1] = amx_ftoc(callback.contactPointA.getY());
			*addr[2] = amx_ftoc(callback.contactPointA.getZ());
			*addr[3] = amx_ftoc(callback.contactPointB.getX());
			*addr[4] = amx_ftoc(callback.contactPointB.getY());
			*addr[5] = amx_ftoc(callback.contactPointB.getZ());
			return amx_ftoc(callback.penetrationDepth);
		}
	}
	return -1;
}

// set user data for dynamic objects
cell AMX_NATIVE_CALL CR_SetDynamicUserData(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		UserData *pUD;
		pUD = (UserData*)rigidBody[params[1]]->col->getUserPointer();
		cell *data;
		amx_GetAddr(amx, params[3], &data);
		switch (params[2])
		{
		case USER_DATA_INT: pUD->m_integerType = *data;
			break;
		case USER_DATA_FLT: pUD->m_floatType = amx_ctof(*data);
			break;
		}
	}
	return 1;
}

// set user data for static objects
cell AMX_NATIVE_CALL CR_SetStaticUserData(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		UserData *pUD = (UserData*)staticBody[params[1]]->col->getUserPointer();
		cell *data;
		amx_GetAddr(amx, params[3], &data);
		switch (params[2])
		{
		case USER_DATA_INT: pUD->m_integerType = params[3];
			break;
		case USER_DATA_FLT: pUD->m_floatType = amx_ctof(params[3]);
			break;
		}
	}
	return 1;
}

// passes integer data for dynamic objects by reference
cell AMX_NATIVE_CALL CR_GetDynamicInt(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		UserData *pUD = (UserData*)rigidBody[params[1]]->col->getUserPointer();
		cell* addr;
		amx_GetAddr(amx, params[2], &addr);
		*addr = pUD->m_integerType;
	}
	return 1;
}

// passes float data for dynamic objects by reference
cell AMX_NATIVE_CALL CR_GetDynamicFloat(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_dynamic[params[1]])
	{
		UserData *pUD = (UserData*)rigidBody[params[1]]->col->getUserPointer();
		cell* addr = NULL;
		amx_GetAddr(amx, params[2], &addr);
		*addr = amx_ftoc(pUD->m_floatType);
	}
	return 1;
}

// passes integer data for static objects by reference
cell AMX_NATIVE_CALL CR_GetStaticInt(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		UserData *pUD = (UserData*)staticBody[params[1]]->col->getUserPointer();
		cell* addr = NULL;
		amx_GetAddr(amx, params[2], &addr);
		*addr = pUD->m_integerType;
	}
	return 1;
}

// passes float data for static objects by reference
cell AMX_NATIVE_CALL CR_GetStaticFloat(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_static[params[1]])
	{
		UserData *pUD = (UserData*)staticBody[params[1]]->col->getUserPointer();
		cell* addr = NULL;
		amx_GetAddr(amx, params[2], &addr);
		*addr = amx_ftoc(pUD->m_floatType);
	}
	return 1;
}

// deletes the user created collision volumes and flushes the memory
cell AMX_NATIVE_CALL CR_FreeMemory(AMX* amx, cell* params)
{
	for (int i = 0; i < CRMAX_BODY; i++)
	{
		if (rigidBody[i])
		{
			index_availability_dynamic[i] = 1;
			SAFE_DELETE(rigidBody[i]);
			//logprintf("Deleted Dynamic id: %i", i);
		}
		if (staticBody[i])
		{
			index_availability_static[i] = 1;
			SAFE_DELETE(staticBody[i]);
			//logprintf("Deleted Static id: %i", i);
		}
	}
	return 1;
}

// returns the time in milliseconds since the plugin has been loaded
cell AMX_NATIVE_CALL CR_GetTimeMilliseconds(AMX* amx, cell* params)
{
	return clock->getTimeMilliseconds();
}

// returns the pool size for static collision volumes
cell AMX_NATIVE_CALL CR_GetStaticPool(AMX* amx, cell* params)
{
	return GetStaticPoolSize();
}

// returns the pool size for dynamic collision volumes
cell AMX_NATIVE_CALL CR_GetDynamicPool(AMX* amx, cell* params)
{
	return GetDynamicPoolSize();
}

// create vehicle collision chassis
cell AMX_NATIVE_CALL CR_CreateVehicleCol(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (index_availability_vehicle[params[1]])
	{
		//thanks to pottus for letting me know the truth about the W component of GTA:SA's quaternion system
		btQuaternion quat(amx_ctof(params[7]), amx_ctof(params[8]), amx_ctof(params[9]), -amx_ctof(params[10]));
		vehicleBody[params[1]] = new VehicleObject(params[1], params[2], params[3], cr_createVehicleCollision(dynamicsWorld, params[3], btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), quat));
		index_availability_vehicle[params[1]] = 0;
	}
	else
		logprintf("cimulator::vehicle collision object is already in use");
	return 1;
}

// removes vehicle collision
cell AMX_NATIVE_CALL CR_RemoveVehicleCol(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (!index_availability_vehicle[params[1]])
	{
		cr_removeVehicleCol(dynamicsWorld, vehicleBody[params[1]]->col); //remove the body from the world
		index_availability_vehicle[params[1]] = 1; //make it available
		SAFE_DELETE(vehicleBody[params[1]]);
		SLEEP(15); //wait...
	}
	return 1;
}

// sets the vehicle position/origin
cell AMX_NATIVE_CALL CR_SetVehicleOrigin(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (index_availability_vehicle[params[1]])
	{
		btTransform transform;
		btVector3 pos(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]));
		transform = vehicleBody[params[1]]->col->getWorldTransform();
		transform.setOrigin(pos);
		vehicleBody[params[1]]->col->setWorldTransform(transform);
	}
	return 1;
}

// passes the vehicle position/origin by reference
cell AMX_NATIVE_CALL CR_GetVehicleOrigin(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (index_availability_vehicle[params[1]])
	{
		cell* addr[3];
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		*addr[0] = amx_ftoc(vehicleBody[params[1]]->col->getWorldTransform().getOrigin().getX());
		*addr[1] = amx_ftoc(vehicleBody[params[1]]->col->getWorldTransform().getOrigin().getY());
		*addr[2] = amx_ftoc(vehicleBody[params[1]]->col->getWorldTransform().getOrigin().getZ());
	}
	return 1;
}

// sets vehicle collision rotation
cell AMX_NATIVE_CALL CR_SetVehicleRotation(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (index_availability_vehicle[params[1]])
	{
		btTransform transform;
		btQuaternion quat(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]), amx_ctof(params[5]));
		transform = vehicleBody[params[1]]->col->getWorldTransform();
		transform.setRotation(quat);
		vehicleBody[params[1]]->col->setWorldTransform(transform);
	}
	return 1;
}

// passe the vehicle collision rotation by reference
cell AMX_NATIVE_CALL CR_GetVehicleRotation(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], 0);
	if (index_availability_vehicle[params[1]])
	{
		cell* addr[4];
		amx_GetAddr(amx, params[2], &addr[0]);
		amx_GetAddr(amx, params[3], &addr[1]);
		amx_GetAddr(amx, params[4], &addr[2]);
		amx_GetAddr(amx, params[5], &addr[3]);
		float x = vehicleBody[params[1]]->col->getWorldTransform().getRotation().getX();
		float y = vehicleBody[params[1]]->col->getWorldTransform().getRotation().getY();
		float z = vehicleBody[params[1]]->col->getWorldTransform().getRotation().getZ();
		float w = vehicleBody[params[1]]->col->getWorldTransform().getRotation().getW();

		*addr[0] = amx_ftoc(x);
		*addr[1] = amx_ftoc(y);
		*addr[2] = amx_ftoc(z);
		*addr[3] = amx_ftoc(w);
	}
	return 1;
}

// index availability check
cell AMX_NATIVE_CALL CR_IsVehicleSlotUsed(AMX* amx, cell* params)
{
	INVALIDITY_CHECK(params[1], -1);
	return (index_availability_vehicle[params[1]] ? 0 : 1);
}

// checks whether a vehicle collision is in contact or not
cell AMX_NATIVE_CALL CR_VehicleContactTest(AMX* amx, cell* params)
{
	btQuaternion rot;
	rot.setEulerZYX(amx_ctof(params[5]), amx_ctof(params[6]), amx_ctof(params[7]));
	btCollisionObject *object;
	CustomCollisionSensor callback;
	object = new btCollisionObject();
	int index = vehicleRef[params[1]];
	object->setCollisionShape((btCompoundShape*)(&vehicle_shapes_dn[index]));
	object->setCollisionFlags(1);
	object->setWorldTransform(btTransform(rot, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]))));
	dynamicsWorld->contactTest(vehicleBody[params[1]]->col, callback);
	delete object;
	return callback.hits;
}

PLUGIN_EXPORT unsigned int PLUGIN_CALL Supports()
{
	return SUPPORTS_VERSION | SUPPORTS_AMX_NATIVES | SUPPORTS_PROCESS_TICK;
}

PLUGIN_EXPORT bool PLUGIN_CALL Load(void **ppData)
{
	pAMXFunctions = ppData[PLUGIN_DATA_AMX_EXPORTS];
	logprintf = (logprintf_t)ppData[PLUGIN_DATA_LOGPRINTF];

	if (cr_readCadb())
		cr_createColObject();
	else
		logprintf("The .cadb file is not present in the scriptfiles directory");

	if (cr_readCCF())
		cr_createColVehicle();

	for (int i = 0; i < CRMAX_BODY; i++)
	{
		index_availability_dynamic[i] = 1; //initializing the availability array
		index_availability_static[i] = 1; //initializing the availability array
		index_availability_vehicle[i] = 1; //initializing the availability array
		rigidBody[i] = NULL;
		staticBody[i] = NULL;
		vehicleBody[i] = NULL;
	}
	clock = new btClock();
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	gContactProcessedCallback = OnCollisionOccur; //assigning function's address to the bullet's internal contactprocessedcallback pointer
	logprintf(" ====================");
	logprintf(" Project: cimulator");
	logprintf(" Version: %0.2f", VERSION);
	logprintf(" Author: codectile");
	logprintf(" Operating System: "OS"");
	logprintf(" ====================");

	return true;
}

PLUGIN_EXPORT void PLUGIN_CALL Unload()
{
	simFlag = 0;
	cr_unLoad(dynamicsWorld); //deletes the collision map
	for (int i = 0; i < CRMAX_BODY; i++)
	{
		if (rigidBody[i])
		{
			index_availability_dynamic[i] = 1;
			SAFE_DELETE(rigidBody[i]);
		}

		if (staticBody[i])
		{
			index_availability_static[i] = 1;
			SAFE_DELETE(staticBody[i]);
		}

		if (vehicleBody[i])
		{
			index_availability_vehicle[i] = 1;
			SAFE_DELETE(vehicleBody[i]);
		}
	}

	delete[] * rigidBody;
	delete[] * staticBody;

	delete[] static_shapes;
	delete[] dynamic_shapes;
	delete[] vehicle_shapes_st;
	delete[] vehicle_shapes_dn;

	delete dynamicsWorld;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;
	delete clock;

	amxList.clear();

	logprintf("\n====================\n");
	logprintf(" Project: cimulator unloaded\n");
	logprintf("\n====================\n");
}

AMX_NATIVE_INFO PluginNatives[] =
{
	//world related natives
	{ "CR_EnableSimulation", CR_EnableSimulation },
	{ "CR_DisableSimulation", CR_DisableSimulation },
	{ "CR_Load", CR_Load },
	{ "CR_RemoveColMap", CR_RemoveColMap },
	{ "CR_RemoveBuilding", CR_RemoveBuilding },
	{ "CR_SetWorldGravity", CR_SetWorldGravity },
	{ "CR_GetWorldGravity", CR_GetWorldGravity },
	{ "CR_RayCastNormal", CR_RayCastNormal },
	{ "CR_RayCast", CR_RayCast },
	{ "CR_RayCastEx", CR_RayCastEx },
	{ "CR_RayCastReflection", CR_RayCastReflection },
	{ "CR_RayCastObjectInfo", CR_RayCastObjectInfo },
	{ "CR_RayCastIntData", CR_RayCastIntData },
	{ "CR_RayCastFloatData", CR_RayCastFloatData },

	//object related functions
	{ "CR_CreateDynamicCol", CR_CreateDynamicCol },
	{ "CR_CreateStaticCol", CR_CreateStaticCol },
	{ "CR_RemoveDynamicCol", CR_RemoveDynamicCol },
	{ "CR_RemoveStaticCol", CR_RemoveStaticCol },
	{ "CR_GetDynamicColModel", CR_GetDynamicColModel },
	{ "CR_GetDynamicColObject", CR_GetDynamicColObject },
	{ "CR_GetStaticColModel", CR_GetStaticColModel },
	{ "CR_GetStaticColObject", CR_GetStaticColObject },
	{ "CR_SetMass", CR_SetMass },
	{ "CR_GetMass", CR_GetMass },
	{ "CR_SetDynamicColShape", CR_SetDynamicColShape },
	{ "CR_SetStaticColShape", CR_SetStaticColShape },
	{ "CR_GetBoundingSphere", CR_GetBoundingSphere },
	{ "CR_GetAABB", CR_GetAABB },
	{ "CR_GetTransform", CR_GetTransform },
	{ "CR_SetDynamicOrigin", CR_SetDynamicOrigin },
	{ "CR_GetDynamicOrigin", CR_GetDynamicOrigin },
	{ "CR_SetStaticOrigin", CR_SetStaticOrigin },
	{ "CR_GetStaticOrigin", CR_GetStaticOrigin },
	{ "CR_SetDynamicRotation", CR_SetDynamicRotation },
	{ "CR_GetDynamicRotation", CR_GetDynamicRotation },
	{ "CR_SetStaticRotation", CR_SetStaticRotation },
	{ "CR_GetStaticRotation", CR_GetStaticRotation },
	{ "CR_SetLinearVelocity", CR_SetLinearVelocity },
	{ "CR_GetLinearVelocity", CR_GetLinearVelocity },
	{ "CR_SetAngularVelocity", CR_SetAngularVelocity },
	{ "CR_GetAngularVelocity", CR_GetAngularVelocity },
	{ "CR_SetTorque", CR_SetTorque },
	{ "CR_SetFriction", CR_SetFriction },
	{ "CR_GetFriction", CR_GetFriction },
	{ "CR_SetRestitution", CR_SetRestitution },
	{ "CR_GetRestitution", CR_GetRestitution },
	{ "CR_IsActive", CR_IsActive },
	{ "CR_IsMoving", CR_IsMoving },
	{ "CR_IsDynamicSlotUsed", CR_IsDynamicSlotUsed },
	{ "CR_IsStaticSlotUsed", CR_IsStaticSlotUsed },
	{ "CR_Activate", CR_Activate },
	{ "CR_GetNumCollisionObject", CR_GetNumCollisionObject },
	{ "CR_GetNumChildShapes", CR_GetNumChildShapes },
	{ "CR_Simulator", CR_Simulator },
	{ "CR_IsCompound", CR_IsCompound },
	{ "CR_Wait", CR_Wait },
	{ "CR_CharacterContactTest", CR_CharacterContactTest },
	{ "CR_DynamicContactTest", CR_DynamicContactTest },
	{ "CR_ContactTestEx", CR_ProxyContactTest },
	{ "CR_GetContactPoints", CR_GetContactPoints },
	{ "CR_SetDynamicUserData", CR_SetDynamicUserData },
	{ "CR_SetStaticUserData", CR_SetStaticUserData },
	{ "CR_GetDynamicInt", CR_GetDynamicInt },
	{ "CR_GetDynamicFloat", CR_GetDynamicFloat },
	{ "CR_GetStaticInt", CR_GetStaticInt },
	{ "CR_GetStaticFloat", CR_GetStaticFloat },
	{ "CR_FreeMemory", CR_FreeMemory },
	{ "CR_GetTimeMilliseconds", CR_GetTimeMilliseconds },
	{ "CR_GetStaticPool", CR_GetStaticPool },
	{ "CR_GetDynamicPool", CR_GetDynamicPool },

	//vehicle related natives
	{ "CR_CreateVehicleCol", CR_CreateVehicleCol },
	{ "CR_RemoveVehicleCol", CR_RemoveVehicleCol },
	{ "CR_SetVehicleOrigin", CR_SetVehicleOrigin },
	{ "CR_GetVehicleOrigin", CR_GetVehicleOrigin },
	{ "CR_SetVehicleRotation", CR_SetVehicleRotation },
	{ "CR_GetVehicleRotation", CR_GetVehicleRotation },
	{ "CR_IsVehicleSlotUsed", CR_IsVehicleSlotUsed },
	{ "CR_VehicleContactTest", CR_VehicleContactTest },
	{ 0, 0 }
};

PLUGIN_EXPORT int PLUGIN_CALL AmxLoad(AMX *amx)
{
	amxList.push_back(amx);
	return amx_Register(amx, PluginNatives, -1);
}


PLUGIN_EXPORT int PLUGIN_CALL AmxUnload(AMX *amx)
{
	return AMX_ERR_NONE;
}

PLUGIN_EXPORT void PLUGIN_CALL ProcessTick()
{
	if (simFlag)
	{
		for (unsigned int i = 0; i < amxList.size(); i++)
		{
			int idx;
			if (!amx_FindPublic(amxList[i], "CR_ProcessSimulation", &idx))
			{
				amx_Exec(amxList[i], NULL, idx);
				Newtime = GetTickCount64();
				cr_simulate(dynamicsWorld, Newtime, Oldtime);
				Oldtime = Newtime;
			}
		}
	}
}

// collision callback, gets called many times a second
bool OnCollisionOccur(btManifoldPoint& cp, void* body0, void* body1)
{
	btCollisionObject* proxy0 = (btCollisionObject*)(body0);
	btCollisionObject* proxy1 = (btCollisionObject*)(body1);
	for (unsigned int i = 0; i < amxList.size(); i++)
	{
		int idx;
		if (!amx_FindPublic(amxList[i], "CR_OnCollisionOccur", &idx))
		{
			amx_Push(amxList[i], proxy1->getUserIndex());
			amx_Push(amxList[i], proxy0->getUserIndex());
			amx_Exec(amxList[i], NULL, idx);
		}
	}
	//printf("\nModelid0: %i", proxy0->getUserIndex());
	//printf("\nModelid1: %i", proxy1->getUserIndex());
	return true;
}

//------------------------- Utility code -------------------------
int GetEmptyDynamicIndex()
{
	for (int i = 0; i < CRMAX_BODY; i++)
		if (index_availability_dynamic[i])
			return i;
	return -1;
}

int GetEmptyStaticIndex()
{
	for (int i = 0; i < CRMAX_BODY; i++)
		if (index_availability_static[i])
			return i;
	return -1;
}

int GetDynamicPoolSize()
{
	int max = 0;
	for (int i = 0; i < CRMAX_BODY; i++)
		if (!index_availability_dynamic[i])
			if (max < i)
				max = i;
	return max;
}

int GetStaticPoolSize()
{
	int max = 0;
	for (int i = 0; i < CRMAX_BODY; i++)
		if (!index_availability_static[i])
			if (max < i)
				max = i;
	return max;
}
//----------------------------------------------------------------
