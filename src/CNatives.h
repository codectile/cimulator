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

#ifndef CNATIVES_H
#define CNATIVES_H

#include "SDK\amx\amx.h"
#include "SDK\plugincommon.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision\CollisionDispatch\btGhostObject.h"
#include "LinearMath\btQuickprof.h"
#include <vector>
#include "CCollision.h"
#include "CDynamics.h"
#define VERSION	1.05
#define SAFE_DELETE(pointer)	if(pointer){ delete pointer; pointer = NULL;}
#ifndef _WIN32
#define SLEEP(ms) usleep(x * 1000)
#define OS	"Linux"
#else
#include <Windows.h>
#define SLEEP(ms)  Sleep(ms)
#define OS "Windows"
#endif // !_WIN32

#define SIMD_DEG_TO_RAD	 btScalar(0.01745329251994329576923690768489)
#define SIMD_RADIAN_TO_DEG	 btScalar(57.295779513082320876798154814105)

#define CRMAX_BODY	30000
#define CRMAX_GHOST	30000

btBroadphaseInterface* broadphase;
btDefaultCollisionConfiguration* collisionConfiguration;
btCollisionDispatcher* dispatcher;
btSequentialImpulseConstraintSolver* solver;
btDiscreteDynamicsWorld* dynamicsWorld;

class DynamicObject
{
public:
	int index;
	int modelid;
	int objectid;
	btRigidBody* col;
	DynamicObject(int idx, int objid, int mdlid, btRigidBody* body) : index(idx), objectid(objid), modelid(mdlid), col(body)
	{
	}
	~DynamicObject()
	{
		index = -1;
		modelid = -1;
		objectid = -1;
		dynamicsWorld->removeRigidBody(col);
		delete col->getMotionState();
		delete col;
		col = NULL;
	}
};

class StaticObject
{
public:
	int index;
	int modelid;
	int objectid;
	btCollisionObject* col;
	StaticObject(int idx, int objid, int mdlid, btCollisionObject* body) : index(idx), objectid(objid), modelid(mdlid), col(body)
	{
	}
	~StaticObject()
	{
		index = -1;
		modelid = -1;
		objectid = -1;
		dynamicsWorld->removeCollisionObject(col);
		delete col;
		col = NULL;
	}
};

class VehicleObject
{
public:
	int index;
	int modelid;
	int vehicleid;
	btCollisionObject* col;

	VehicleObject(int idx, int vhclid, int mdlid, btCollisionObject* body) : index(idx), vehicleid(vhclid), modelid(mdlid), col(body)
	{
	}

	~VehicleObject()
	{
		index = -1;
		vehicleid = -1;
		modelid = -1;
		delete col;
		col = NULL;
	}
};

DynamicObject* rigidBody[CRMAX_BODY];
StaticObject* staticBody[CRMAX_BODY];
VehicleObject* vehicleBody[CRMAX_BODY];
btGhostObject* ghost[CRMAX_GHOST];

btClock* clock;

typedef void(*logprintf_t)(char* format, ...);


logprintf_t logprintf;
extern void *pAMXFunctions;


std::vector<AMX*> amxList;
static int index_availability_static[CRMAX_BODY];
static int index_availability_dynamic[CRMAX_BODY];
static int index_availability_vehicle[CRMAX_BODY];
static int simFlag = 0;

//declaration
bool OnCollisionOccur(btManifoldPoint& cp, void* body0, void* body1);
#endif // !CNATIVES_H
