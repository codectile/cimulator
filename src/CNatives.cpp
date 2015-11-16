/*
cimulator plugin for SanAndreas Multiplayer
Copyright (c) 2015 codectile

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
cell AMX_NATIVE_CALL CR_RayTraceNormal(AMX* amx, cell* params)
{
	btVector3 normal;
	if(cr_rayTraceNormal(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), normal))
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
cell AMX_NATIVE_CALL CR_RayTrace(AMX* amx, cell* params)
{
	btVector3 normal;
	if(cr_rayTrace(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), normal))
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
cell AMX_NATIVE_CALL CR_RayTraceEx(AMX* amx, cell* params)
{
	btVector3 hitpoint;
	int modelid;
	if (cr_rayTraceEx(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), hitpoint, modelid))
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
cell AMX_NATIVE_CALL CR_RayTraceReflection(AMX* amx, cell* params)
{
	btVector3 reflection;
	btScalar angle;
	if (cr_rayTraceReflection(dynamicsWorld, btVector3(amx_ctof(params[1]), amx_ctof(params[2]), amx_ctof(params[3])), btVector3(amx_ctof(params[4]), amx_ctof(params[5]), amx_ctof(params[6])), reflection, angle))
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

// creates collision volume
cell AMX_NATIVE_CALL CR_CreateColVolume(AMX* amx, cell* params)
{
	rigidBody[params[1]] = new colObject(params[1], params[2], params[3], cr_addColBody(dynamicsWorld, params[3], amx_ctof(params[4]), btVector3(amx_ctof(params[5]), amx_ctof(params[6]), amx_ctof(params[7])), btVector3(amx_ctof(params[8]), amx_ctof(params[9]), amx_ctof(params[10])), params[11], params[12]));
	rigidBody[params[1]]->col->setUserIndex(params[3]); //set the modelid
	//col_cache.push_back(rigidBody[params[1]]); //push the object to the vector for deletion
	index_availability[params[1]] = 0; //make it unavailable
	return 1;
}

// removes the collision volume from the world
cell AMX_NATIVE_CALL CR_RemoveColVolume(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
	{
		cr_removeColBody(dynamicsWorld, rigidBody[params[1]]->col); //remove the body from the world
		index_availability[params[1]] = 1; //make it available
		SAFE_DELETE(rigidBody[params[1]]);
		SLEEP(15); //wait...
	}
	return 1;
}

// get modelid
cell AMX_NATIVE_CALL CR_GetColVolumeModel(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		return rigidBody[params[1]]->modelid; //return the modelid
	return -1;
}

// get object id
cell AMX_NATIVE_CALL CR_GetColVolumeObjectID(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		return rigidBody[params[1]]->objectid; //return the objectid
	return -1;
}

// sets mass
cell AMX_NATIVE_CALL CR_SetMass(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setMass(rigidBody[params[1]]->col, amx_ctof(params[2])); //set the mass
	return 1;
}

// returns mass
cell AMX_NATIVE_CALL CR_GetMass(AMX* amx, cell* params)
{
	btScalar mass = -1.f;
	if (!index_availability[params[1]])
		mass = cr_getMass(rigidBody[params[1]]->col); //returns the mass
	return amx_ftoc(mass);
}

// sets a collision shape (real-time)
cell AMX_NATIVE_CALL CR_SetCollisionShape(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setCollisionShape(rigidBody[params[1]]->col, params[2]); //replaces the collision shape at real-time
	return 1;
}

// returns bounding sphere
cell AMX_NATIVE_CALL CR_GetBoundingSphere(AMX* amx, cell* params)
{
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
	if (!index_availability[params[1]])
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

// sets origin
cell AMX_NATIVE_CALL CR_SetOrigin(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setOrigin(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]))); //sets the origin
	return 1;
}

// returns origin
cell AMX_NATIVE_CALL CR_GetOrigin(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
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

// sets rotations
cell AMX_NATIVE_CALL CR_SetRotation(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setRotation(rigidBody[params[1]]->col, amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4]));
	return 1;
}

// passes rotations by reference
cell AMX_NATIVE_CALL CR_GetRotation(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
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

//sets linear velocity
cell AMX_NATIVE_CALL CR_SetLinearVelocity(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setLinearVelocity(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])));
	return 1;
}

// passes linear velocity by reference
cell AMX_NATIVE_CALL CR_GetLinearVelocity(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
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
	if (!index_availability[params[1]])
		cr_setAngularVelocity(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])));
	return 1;
}

// passes angular velocity by reference
cell AMX_NATIVE_CALL CR_GetAngularVelocity(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
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
	if (!index_availability[params[1]])
		cr_setTorque(rigidBody[params[1]]->col, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])));
	return 1;
}

// sets friction of a collision volume
cell AMX_NATIVE_CALL CR_SetFriction(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setFriction(rigidBody[params[1]]->col, amx_ctof(params[2]));
	return 1;
}

// returns the friction of a collision volume
cell AMX_NATIVE_CALL CR_GetFriction(AMX* amx, cell* params)
{
	btScalar friction = 0.f;
	if (!index_availability[params[1]])
		friction = cr_getFriction(rigidBody[params[1]]->col);
	return amx_ftoc(friction);
}

// sets the coefficient of restitution of the collision volume (avoid using it)
cell AMX_NATIVE_CALL CR_SetRestitution(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		cr_setRestitution(rigidBody[params[1]]->col, amx_ctof(params[2]));
	return 1;
}

// returns coefficient of restitution of the collision volume
cell AMX_NATIVE_CALL CR_GetRestitution(AMX* amx, cell* params)
{
	btScalar rest = 0.f;
	if (!index_availability[params[1]])
		rest = cr_getRestitution(rigidBody[params[1]]->col);
	return amx_ftoc(rest);
}

//checks whether a collision volume is active or not
cell AMX_NATIVE_CALL CR_IsActive(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		return cr_isActive(rigidBody[params[1]]->col);
	return 0;
}

// checks whether a collision volume is static or dynamic
cell AMX_NATIVE_CALL CR_IsStatic(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
		if (rigidBody[params[1]]->col->isStaticObject())
			return 1;
	return 0;
}

// checks if the specified slot is available
cell AMX_NATIVE_CALL CR_IsIndexAvailable(AMX* amx, cell* params)
{
	return index_availability[params[1]];
}

// activates a deactivated collision volume
cell AMX_NATIVE_CALL CR_Activate(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
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
	delete ob->getCollisionShape();
	delete ob;
	return sensor.hit;
}

// creates a ghost object which is capable of recording collisions
cell AMX_NATIVE_CALL CR_CreateGhost(AMX* amx, cell* params)
{
	if (ghost[params[1]] != NULL)
		dynamicsWorld->removeCollisionObject(ghost[params[1]]);
	ghost[params[1]] = cr_createGhost(dynamicsWorld, params[2], btVector3(amx_ctof(params[3]), amx_ctof(params[4]), amx_ctof(params[5])), btVector3(amx_ctof(params[6]), amx_ctof(params[7]), amx_ctof(params[8])));
	return 1;
}

// returns number of objects which overlaps the AABB of the created ghost object
cell AMX_NATIVE_CALL CR_GetNumOverlappingObjects(AMX* amx, cell* params)
{
	if(ghost[params[1]] != NULL)
		return cr_getNumOverlappingObjects(ghost[params[1]]);
	return -1;
}

// checks collision for a given collision volume
cell AMX_NATIVE_CALL CR_ContactTest(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
	{
		CustomCollisionSensor callback;
		dynamicsWorld->updateSingleAabb(rigidBody[params[1]]->col);
		dynamicsWorld->contactTest(rigidBody[params[1]]->col, callback);
		return callback.hit;
	}
	return 0;
}

// checks collision for a given modelid
cell AMX_NATIVE_CALL CR_ContactTestEx(AMX* amx, cell* params)
{
	CustomCollisionSensor callback;
	btRigidBody* body = cr_addColBody(dynamicsWorld, params[1], 0.f, btVector3(amx_ctof(params[2]), amx_ctof(params[3]), amx_ctof(params[4])), btVector3(amx_ctof(params[5]), amx_ctof(params[6]), amx_ctof(params[7])), 0, ACTIVE_TAG);
	dynamicsWorld->contactTest(body, callback);
	cr_removeColBody(dynamicsWorld, body);
	delete body->getMotionState();
	delete body;
	return callback.hit;
}

// point of contact of two objects are passed by reference
cell AMX_NATIVE_CALL CR_GetContactPoints(AMX* amx, cell* params)
{
	if (!index_availability[params[1]])
	{
		CustomCollisionSensor callback;
		dynamicsWorld->contactTest(rigidBody[params[1]]->col, callback);
		if (callback.hit)
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

// deletes the user created collision volumes and flushes the memory
cell AMX_NATIVE_CALL CR_FreeMemory(AMX* amx, cell* params)
{
	for (int i = 0; i < CRMAX_BODY; i++)
	{
		if (rigidBody[i] != NULL)
		{
			index_availability[i] = 1;
			SAFE_DELETE(rigidBody[i]);
			//logprintf("Deleted id: %i", i);
		}
	}
	return 1;
}

// returns the time in milliseconds since the plugin has been loaded
cell AMX_NATIVE_CALL CR_GetTimeMilliseconds(AMX* amx, cell* params)
{
	return clock->getTimeMilliseconds();
}

PLUGIN_EXPORT unsigned int PLUGIN_CALL Supports()
{
	return SUPPORTS_VERSION | SUPPORTS_AMX_NATIVES | SUPPORTS_PROCESS_TICK;
}

PLUGIN_EXPORT bool PLUGIN_CALL Load(void **ppData)
{
	pAMXFunctions = ppData[PLUGIN_DATA_AMX_EXPORTS];
	logprintf = (logprintf_t)ppData[PLUGIN_DATA_LOGPRINTF];
	for (int i = 0; i < CRMAX_BODY; i++)
	{
		index_availability[i] = 1; //initializing the availability array
		rigidBody[i] = NULL;
	}
	clock = new btClock();
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	cr_readCadb();
	cr_createColObject();
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
	logprintf("\n====================\n");
	logprintf(" Project: cimulator unloaded\n");
	logprintf("\n====================\n");

	cr_unLoad(dynamicsWorld); //deletes the collision map
	for (int i = 0; i < CRMAX_BODY; i++)
	{
		if (rigidBody[i] != NULL)
		{
			index_availability[i] = 1;
			SAFE_DELETE(rigidBody[i]);
		}
	}
	delete dynamicsWorld;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;
	delete clock;

	delete[] *rigidBody;
	delete[] *ghost;
}

AMX_NATIVE_INFO PluginNatives[] =
{
	{ "CR_EnableSimulation", CR_EnableSimulation },
	{ "CR_DisableSimulation", CR_DisableSimulation },
	{ "CR_Load", CR_Load },
	{ "CR_RemoveColMap", CR_RemoveColMap },
	{ "CR_SetWorldGravity", CR_SetWorldGravity },
	{ "CR_GetWorldGravity", CR_GetWorldGravity },
	{ "CR_RayTraceNormal", CR_RayTraceNormal },
	{ "CR_RayTrace", CR_RayTrace },
	{ "CR_RayTraceEx", CR_RayTraceEx },
	{ "CR_RayTraceReflection", CR_RayTraceReflection },
	{ "CR_CreateColVolume", CR_CreateColVolume },
	{ "CR_RemoveColVolume", CR_RemoveColVolume },
	{ "CR_GetColVolumeModel", CR_GetColVolumeModel },
	{ "CR_GetColVolumeObjectID", CR_GetColVolumeObjectID },
	{ "CR_SetMass", CR_SetMass },
	{ "CR_GetMass", CR_GetMass },
	{ "CR_SetCollisionShape", CR_SetCollisionShape },
	{ "CR_GetBoundingSphere", CR_GetBoundingSphere },
	{ "CR_GetAABB", CR_GetAABB },
	{ "CR_GetTransform", CR_GetTransform },
	{ "CR_SetOrigin", CR_SetOrigin },
	{ "CR_GetOrigin", CR_GetOrigin },
	{ "CR_SetRotation", CR_SetRotation },
	{ "CR_GetRotation", CR_GetRotation },
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
	{ "CR_IsStatic", CR_IsStatic },
	{ "CR_IsIndexAvailable", CR_IsIndexAvailable },
	{ "CR_Activate", CR_Activate },
	{ "CR_GetNumCollisionObject", CR_GetNumCollisionObject },
	{ "CR_GetNumChildShapes", CR_GetNumChildShapes },
	{ "CR_Simulator", CR_Simulator },
	{ "CR_IsCompound", CR_IsCompound },
	{ "CR_Wait", CR_Wait },
	{ "CR_CharacterContactTest", CR_CharacterContactTest },
	{ "CR_CreateGhost", CR_CreateGhost },
	{ "CR_GetNumOverlappingObjects", CR_GetNumOverlappingObjects },
	{ "CR_ContactTest", CR_ContactTest },
	{ "CR_ContactTestEx", CR_ContactTestEx },
	{ "CR_GetContactPoints", CR_GetContactPoints },
	{ "CR_FreeMemory", CR_FreeMemory },
	{ "CR_GetTimeMilliseconds", CR_GetTimeMilliseconds },
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
				amx_Exec(amxList[i], NULL, idx);
		}
	}
}
