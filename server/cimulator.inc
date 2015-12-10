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

#pragma library	cimulator

#define	CR_MAX_DYNAMIC	1000	//set it according to your own need, the max limit is 30000
#define	CR_MAX_STATIC	1000	//set it according to your own need, the max limit is 30000


#define ACTIVE_TAG	1 //property by which the collision volume can be activated again
#define	DISABLE_DEACTIVATION	2 //will simulate for ever, increases cpu stress
#define DISABLE_SIMULATION	3 //deactivates forever
#define	ISLAND_SLEEPING	4 //bullet automatically does this so no need of using it
#define	WANTS_DEACTIVATION	5 //tries to make the collision volume to go off to sleep

static cr_newtime, cr_oldtime;


/* Natives List:
 
 * Static Object Natives

		CR_CreateStaticCol
		CR_RemoveStaticCol
		CR_GetStaticColModel
		CR_GetStaticColObject
		CR_SetStaticColShape
		CR_SetStaticOrigin
		CR_GetStaticOrigin
		CR_SetStaticRotation
		CR_GetStaticRotation
		CR_IsStaticIdxAvailable
		CR_StaticContactTest

  * Dynamic Object Natives

		CR_CreateDynamicCol
		CR_RemoveDynamicCol
		CR_GetDynamicColModel
		CR_GetDynamicColObject
		CR_SetDynamicColShape
		CR_SetDynamicOrigin
		CR_GetDynamicOrigin
		CR_SetDynamicRotation
		CR_GetDynamicRotation
		CR_IsDynamicIdxAvailable
		CR_DynamicContactTest
		CR_SetLinearVelocity
		CR_GetLinearVelocity
		CR_SetAngularVelocity
		CR_GetAngularVelocity
		CR_SetTorque
		CR_SetFriction
		CR_GetFriction
		CR_SetRestitution
		CR_GetRestitution
		CR_IsActive
		CR_Activate
		CR_IsMoving
		CR_SetMass
		CR_GetMass
		CR_GetTransform
		CR_GetContactPoints

  * World Natives
		
		CR_EnableSimulation
		CR_DisableSimulation
		CR_Load
		CR_RemoveColMap
		CR_SetWorldGravity
		CR_GetWorldGravity
		CR_RayTraceNormal
		CR_RayTrace
		CR_RayTraceEx
		CR_RayTraceReflection
		CR_GetBoundingSphere
		CR_GetAABB
		CR_GetNumCollisionObject
		CR_GetNumChildShapes
		CR_Simulator
		CR_IsCompound
		CR_Wait
		CR_CharacterContactTest
		CR_ContactTestEx
		CR_FreeMemory
		CR_GetTimeMilliseconds
*/

/*
 * stops the simulation
 */
native CR_EnableSimulation();

/*
 * starts the simulation
 */
native CR_DisableSimulation();

/*
 * loads the collision map, here the parameter worldrest mean the coefficienct of restitution of the world
 * avoid using restitution always keep it zero
 */
native CR_Load(Float:worldrest = 0.0);

/*
 * removes the collision map
 */
 native CR_RemoveColMap();

/*
 * sets the world gravity
 */
native CR_SetWorldGravity(Float:x, Float:y, Float:z); 

/*
 * returns the world gravity
 */
native CR_GetWorldGravity(&Float:x, &Float:y, &Float:z); 

/*
 * shoots an invisible ray to a specified destination, returns the collision point if collided
 */
native CR_RayTraceNormal(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3);

/*
 * same as above, returns the normal of the hit point
 */
native CR_RayTrace(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3);

/*
 * same as above, modelid is passed by reference.
 */
 native CR_RayTraceEx(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3, &modelid);

 /*
  * passes the reflection vector and the angle by reference
  */
native CR_RayTraceReflection(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:rx, &Float:ry, &Float:rz, &Float:angle);

/*
 * frees all the user created memory i.e. collision volumes
 * used internally
 */
 native CR_FreeMemory();

/*
 * removes dynamic collision volume from the world
 * used internally
 */
 native CR_RemoveDynamicCol(index);

 /*
 * removes static collision volume from the world
 * used internally
 */
 native CR_RemoveStaticCol(index);

/*
 * creates a dynamic collision volume, set inertia  = 1 to use inertia, for tags look to their definitions
 */
native CR_CreateDynamicCol(index, objectid, modelid, Float:mass, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll, tag = ACTIVE_TAG);

/*
 * creates a static collision volume
 */
native CR_CreateStaticCol(index, objectid, modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll);

/*
 * returns modelid of the dynamic collision volume
 */
 native CR_GetDynamicColModel(index);

 /*
  * returns modelid of the static collision volume
  */
 native CR_GetStaticColModel(index);

/*
 * sets the mass of the collision volume
 */
 native CR_SetMass(index, Float:mass);

 /*
 * returns the mass of the collision volume
 */
 native Float:CR_GetMass(index);

/*
 * the most useful function yet, this function changes the collision shape of collision volume, in real-time.
 * The dynamic collision volume change its collision shape to that of the specified modelid.
 */
native CR_SetDynamicColShape(index, modelid);

/*
 * the most useful function yet, this function changes the collision shape of collision volume, in real-time.
 * The static collision volume change its collision shape to that of the specified modelid.
 */
native CR_SetStaticColShape(index, modelid);

/*
 * passes the offset and radius of the model's bounding sphere by reference
 */
native CR_GetBoundingSphere(modelid, &Float:x, &Float:y, &Float:z, &Float:radius);

/*
 * returns thr axi-aligned bounding box of the modelid
 */
native CR_GetAABB(modelid, &Float:minx, &Float:miny, &Float:minz, &Float:maxx, &Float:maxy, &Float:maxz);

/*
 * returns the position and rotation of collision volume
 */
native CR_GetTransform(index, &Float:x, &Float:y, &Float:z, &Float:rx, &Float:ry, &Float:rz);

/*
 * set the position of the dynamic collision volume
 */
native CR_SetDynamicOrigin(index, Float:x, Float:y, Float:z);

/*
 * set the position of the static collision volume
 */
native CR_SetStaticOrigin(index, Float:x, Float:y, Float:z);

/*
 * returns the position of the dynamic collision volume
 */
native CR_GetDynamicOrigin(index, &Float:x, &Float:y, &Float:z);

/*
 * returns the position of the static collision volume
 */
native CR_GetStaticOrigin(index, &Float:x, &Float:y, &Float:z);

/*
 * sets the rotation of dynamic collision colume
 */
native CR_SetDynamicRotation(index, Float:yaw, Float:pitch, Float:roll);

/*
 * sets the rotation of static collision colume
 */
native CR_SetStaticRotation(index, Float:yaw, Float:pitch, Float:roll);

/*
 * returns the rotation of dynamic collision colume
 */
native CR_GetDynamicRotation(index, &Float:yaw, &Float:pitch, &Float:roll);

/*
 * returns the rotation of static collision colume
 */
native CR_GetStaticRotation(index, &Float:yaw, &Float:pitch, &Float:roll);

/*
 * sets the linear velocity of the collision volume
 */
native CR_SetLinearVelocity(index, Float:vx, Float:vy, Float:vz);

/*
 * returns the linear velocity of the collision volume
 */
native CR_GetLinearVelocity(index, &Float:vx, &Float:vy, &Float:vz);

/* 
 * sets the angular velocity of the collision volume
 */
native CR_SetAngularVelocity(index, Float:vx, Float:vy, Float:vz);

/*
 * returns the angular velocity of the collision volume
 */
native CR_GetAngularVelocity(index, &Float:vx, &Float:vy, &Float:vz);

/*
 * applies torgue to the body
 */
native CR_SetTorque(index, Float:tx, Float:ty, Float:tz);

/*
 * sets the coefficient of friction of the collision volume
 */
native CR_SetFriction(index, Float:friction);

/*
 * returns the coefficient of friction of the collision volume
 */
native Float:CR_GetFriction(index);

/*
 * sets the coefficient of restitution of the collision volume
 */
native CR_SetRestitution(index, Float:rest);

/*
 * returns the coefficient of restitution of the collision volume
 */
native Float:CR_GetRestitution(index);

/*
 * checks whether a dynamic index is available or not
 */
 native CR_IsDynamicIdxAvailable(index);

 /*
 * checks whether a static index is available or not
 */
 native CR_IsStaticIdxAvailable(index);

/*
 * checks whether is the collision volume is active or not
 * returns 1 if true else 0.
 */
native CR_IsActive(index);

/*
 * checks whether a dynamic collision volume is moving or not
 * returns 1 if moving else 0.
 */
native CR_IsMoving(index);

/*
 * activates the collision volume
 */
native CR_Activate(index);

/*
 * the heart of this plugin, simulates the collision volumes.
 */
native CR_Simulator(newtime, oldtime);

/* 
 * returns the number of collision volumes created by the plugin(including map)
 */
native CR_GetNumCollisionObject();

/*
 * returns the number of chil shapes, contained in the collision shape of the specified modelid
 * In GTA each collision shape contain 1 or more than one sub collision shapes and together the make a compound shape.
 * So the sub shapes in a compound shape is known as child shapes.
 */
native CR_GetNumChildShapes(modelid);

/*
 * checks if the collision shaoe of the modelid is compound or not
 */
native CR_IsCompound(modelid);

/* 
 * pause the execution of the plugin by specified time in milliseconds.
 */
native CR_Wait(ms);

/*
 * creates a ghost objects which records it overlapping objects
 */
native CR_CreateGhost(ghostIndex, modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll);

/*
 * returns number of overlapping objects of the created ghost object
 */
native CR_GetNumOverlappingObjects(ghostIndex);

/*
 * checks whether a dynamic collision volume is colliding or not
 */
native CR_DynamicContactTest(index);

/*
 * checks whether a static collision volume is colliding or not
 */
native CR_StaticContactTest(index);

/*
 * predicts whether a modelid will be colliding at a specific orientation or not
 */
native CR_ContactTestEx(modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll);

/*
 * creates a player collision approximation and checks whether it is colliding or not
 */
native CR_CharacterContactTest(Float:x, Float:y, Float:z, Float:radius, Float:height);

/*
 * provides contact points of the collision volumes
 * x1, y1, z1 contact point on the specified collision volume(index)
 * x2, y2, z2 contact point on the last collided object 
 * returns penetration depth
 */
native Float:CR_GetContactPoints(index, &Float:x1, &Float:y1, &Float:z1, &Float:x2, &Float:y2, &Float:z2);

/*
 * returns the time in milliseconds since cimulator was initialized
 */
native CR_GetTimeMilliseconds();

 /*
  * returns the objectid with respect to the dynamic index
  */
native CR_GetDynamicColObject(index);

/*
  * returns the objectid with respect to the dynamic index
  */
native CR_GetStaticColObject(index);

/*
 * sets the tansform of the collision volume and the object passed
 */
stock CR_SetTransform(index, Float:x, Float:y, Float:z, Float:rotx, Float:roty, Float:rotz)
{
	new ob = CR_GetDynamicColObject(index);
	CR_SetDynamicOrigin(index, x, y, z);
	CR_SetDynamicRotation(index, rotx, roty, rotz);
	SetDynamicObjectPos(ob, x, y, z);
	SetDynamicObjectRot(ob, rotx, roty, rotz);
}

/*
 * updates the transform of the objects with respect to the collision volume's transform.
 */
stock CR_UpdateTransform(index)
{
	new ob = CR_GetDynamicColObject(index);
	if(IsValidDynamicObject(ob) && CR_IsActive(index) && CR_IsMoving(index))
	{
		new Float:x, Float:y, Float:z, Float:rotx, Float:roty, Float:rotz;
		CR_GetTransform(index, x, y, z, rotx, roty, rotz);
		SetDynamicObjectPos(ob, x, y, z);
		SetDynamicObjectRot(ob, rotx, roty, rotz);
		CR_Wait(0);
	}
}

/*
 * processes all objects and volumes
 */
stock CR_ProcessItems()
{
	for(new i = 0; i < CR_MAX_DYNAMIC; i++)
		CR_UpdateTransform(i);
}

/*
 * deletes all user created objects hooked with collision volumes
 */
stock CR_DestroyAllColVolumes()
{
	for(new i = 0; i < CR_MAX_DYNAMIC; i++)
	{
	    if(IsValidDynamicObject(CR_GetDynamicColObject(i)))
		{
			DestroyDynamicObject(CR_GetDynamicColObject(i));
			CR_RemoveDynamicCol(i);
		}
	}

	for(new i = 0; i < CR_MAX_STATIC; i++)
	{
	    if(IsValidDynamicObject(CR_GetStaticColObject(i)))
		{
			DestroyDynamicObject(CR_GetStaticColObject(i));
			CR_RemoveStaticCol(i);
		}
	}
}

/*
 * deletes dynamic objects along with collision volumes
 */
stock CR_DestroyDynamicCol(index)
{
	new ob = CR_GetDynamicColObject(index);
	if(IsValidDynamicObject(ob))
	{
		DestroyDynamicObject(ob);
		CR_RemoveDynamicCol(index);
	}
}

/*
 * deletes static objects along with collision volumes
 */
stock CR_DestroyStaticCol(index)
{
	new ob = CR_GetStaticColObject(index);
	if(IsValidDynamicObject(ob))
	{
		DestroyDynamicObject(ob);
		CR_RemoveStaticCol(index);
	}
}

/*
 * deletes all streamer dynamic objects and collision volumes, plus flushes the memory
 */
stock CR_FlushMemory()
{
	for(new i = 0; i < CR_MAX_DYNAMIC; i++)
		if(IsValidDynamicObject(CR_GetDynamicColObject(i)))
			DestroyDynamicObject(CR_GetDynamicColObject(i));
	for(new i = 0; i < CR_MAX_STATIC; i++)
		if(IsValidDynamicObject(CR_GetStaticColObject(i)))
			DestroyDynamicObject(CR_GetStaticColObject(i));
	CR_FreeMemory();
}

/*
 * Dynamic server-synced Simulation
 */
forward CR_ProcessSimulation();
public	CR_ProcessSimulation()
{
	CR_ProcessItems();
	cr_newtime = GetTickCount();
	CR_Simulator(cr_newtime, cr_oldtime);
	cr_oldtime = cr_newtime;
	//SetTimer("CR_ProcessSimulation", 20, false); now the simulation syncs with the server
}

/*
 * hooking OnGameModeInit()
 */
public OnGameModeInit()
{
	cr_newtime = cr_oldtime = GetTickCount();
    #if defined CR_OnGameModeInit
        CR_OnGameModeInit();
    #endif
    return 1;
}
#if defined _ALS_OnGameModeInit
    #undef OnGameModeInit
#else
    #define _ALS_OnGameModeInit
#endif
#define OnGameModeInit CR_OnGameModeInit
#if defined CR_OnGameModeInit
    forward CR_OnGameModeInit();
#endif
