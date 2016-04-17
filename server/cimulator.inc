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

#pragma library	cimulator

#define ACTIVE_TAG	1 //property by which the collision volume can be activated again
#define	DISABLE_DEACTIVATION	2 //will simulate for ever, increases cpu stress
#define DISABLE_SIMULATION	3 //deactivates forever
#define	ISLAND_SLEEPING	4 //bullet automatically does this so no need of using it
#define	WANTS_DEACTIVATION	5 //tries to make the collision volume to go off to sleep

#define USER_DATA_INT	1 //integer type
#define USER_DATA_FLT	2 //float type

#if !defined IsValidVehicle
native IsValidVehicle(vehicleid);
#endif

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
 * removes the collision volumes for a given range and modelid
 * To remove all models in a given range, use -1 as modelid in the function below
 * The function must be used used after loading the map i.e. after using CR_Load
 */
 native CR_RemoveBuilding(modelid, Float:x, Float:y, Float:z, Float:range);

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
native CR_RayCastNormal(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3);

/*
 * same as above, returns the normal of the hit point
 */
native CR_RayCast(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3);

/*
 * same as above, modelid is passed by reference.
 */
 native CR_RayCastEx(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3, &modelid);

/*
 * passes the reflection vector and the angle by reference
 */
native CR_RayCastReflection(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:rx, &Float:ry, &Float:rz, &Float:angle);

/*
 * passes the object's information by reference which the object hits
 */
native CR_RayCastObjectInfo(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &modelid, &Float:boundingRadius, &isStatic);

/*
 * returns the hitpoint and the integer object data set by the user
 */
native CR_RayCastIntData(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3, &data);

/*
 * returns the hitpoint and the float object data set by the user
 */
native CR_RayCastFloatData(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x3, &Float:y3, &Float:z3, &Float:data);

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
 * inertia is set to 0 by default, it's good for roleplaying servers if you only want to simulate tossing of objects
 * Here yaw is the z axis, pitch is the y axis, roll is the x axis
 * return value: index/collision id
 */
native CR_CreateDynamicCol(objectid, modelid, Float:mass, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll, inertia = 0, tag = ACTIVE_TAG);

/*
 * creates a static collision volume
 * Here yaw is the z axis, pitch is the y axis, roll is the x axis
 * return value: index/collision id
 */
native CR_CreateStaticCol(objectid, modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll);

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
 * returns the axi-aligned bounding box of the modelid
 */
native CR_GetAABB(modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll, &Float:minx, &Float:miny, &Float:minz, &Float:maxx, &Float:maxy, &Float:maxz);

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
 native CR_IsDynamicSlotUsed(index);

 /*
 * checks whether a static index is available or not
 */
 native CR_IsStaticSlotUsed(index);

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
 * checks whether a dynamic collision volume is colliding or not
 */
native CR_DynamicContactTest(index);

/*
 * checks whether a static collision volume is colliding or not
 * Removed( version 1.07 ), use CR_ContactTestEx instead
 */
//native CR_StaticContactTest(index);

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
 * Sets the data according to the user for a dynamic collision volume
 * Pass only one value in the third parameter, values after the third argument won't be considered
 * Set the type parameter to USER_DATA_INT/USER_DATA_FLT for storing an integer/float value
 */
native CR_SetDynamicUserData(index, type, {Float,_}:...);

/*
 * Sets the data according to the user for a static collision volume
 * Pass only one value in the third parameter, values after the third argument won't be considered
 * Set the type parameter to USER_DATA_INT/USER_DATA_FLT for storing an integer/float value
 */
native CR_SetStaticUserData(index, type, {Float,_}:...);

/*
 * passes the integer data set by the user for a dynamic collision volume by reference
 */
native CR_GetDynamicInt(index, &data);

/*
 * passes the float data set by the user for a dynamic collision volume by reference
 */
native CR_GetDynamicFloat(index, &Float:data);

/*
 * passes the integer data set by the user for a static collision volume by reference
 */
native CR_GetStaticInt(index, &data);

/*
 * passes the float data set by the user for a static collision volume by reference
 */
native CR_GetStaticFloat(index, &Float:data);

/*
 * returns the pool size for static collision volumes
 */
native CR_GetStaticPool();

/*
 * returns the pool size for dynamic collision volumes
 */
native CR_GetDynamicPool();

/*
 * used internally for object placement
 */
native Float:CR_PlacementCorrection(modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll);

/*
 * creates precise vehicle collision
 */
native CR_CreateVehicleCol(vehicleid, modelid, Float:x, Float:y, Float:z, Float:qx, Float:qy, Float:qz, Float:qw);

/*
 * deletes the vehicle collision
 */
native CR_RemoveVehicleCol(index);

/*
 * sets the position of the collision volume of the vehicle
 */
native CR_SetVehicleOrigin(index, Float:x, Float:y, Float:z);

/*
 * passes the position by reference
 */
native CR_GetVehicleOrigin(index, &Float:x, &Float:y, &Float:z);

/*
 * sets the rotation of the collision volume
 */
native CR_SetVehicleRotation(index, Float:qx, Float:qy, Float:qz, Float:qw);

/*
 * passes the rotation by reference
 */
native CR_GetVehicleRotation(index, &Float:qx, &Float:qy, &Float:qz, &Float:qw);

/*
 * returns the vehicle id hooked to the vehicle collision volume
 */
native CR_GetVehicleID(index);

/*
 * returns the model id hooked to the vehicle collision volume
 */
native CR_GetVehicleModel(index);

/*
 * verify whether the index is available for creation of collision volume
 */
native CR_IsVehicleSlotUsed(index);

/*
 * checks whether the vehicle is in contact with any other object/world
 */
native CR_VehicleContactTest(modelid, Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll);

/*
 * returns the highest valid vehicle collision id 
 */
native CR_GetVehiclePool();

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
		//CR_Wait(0);
	}
}

/*
 * deletes all user created objects hooked with collision volumes
 * to perform the deletion of vehicles and their collision volumes as well,
 * set the vehicle parameter to 1
 */
stock CR_DestroyAllColVolumes(vehicle = 0)
{
	for(new i = 0; i <= CR_GetDynamicPool(); i++)
	{
	    if(IsValidDynamicObject(CR_GetDynamicColObject(i)))
		{
			DestroyDynamicObject(CR_GetDynamicColObject(i));
			CR_RemoveDynamicCol(i);
		}
	}

	for(new i = 0; i <= CR_GetStaticPool(); i++)
	{
	    if(IsValidDynamicObject(CR_GetStaticColObject(i)))
		{
			DestroyDynamicObject(CR_GetStaticColObject(i));
			CR_RemoveStaticCol(i);
		}
	}

	if(vehicle)
	{
		for(new i = 0; i <= CR_GetStaticPool(); i++)
		{
			if(IsValidVehicle(CR_GetVehicleID(i)))
			{
				DestroyVehicle(CR_GetVehicleID(i));
				CR_RemoveVehicleCol(i);
			}
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
	for(new i = 0; i <= CR_GetDynamicPool(); i++)
		if(IsValidDynamicObject(CR_GetDynamicColObject(i)))
			DestroyDynamicObject(CR_GetDynamicColObject(i));
	for(new i = 0; i <= CR_GetStaticPool(); i++)
		if(IsValidDynamicObject(CR_GetStaticColObject(i)))
			DestroyDynamicObject(CR_GetStaticColObject(i));
	for(new i = 0; i <= CR_GetVehiclePool(); i++)
		if(IsValidVehicle(CR_GetVehicleID(i)))
			DestroyVehicle(CR_GetVehicleID(i));
	CR_FreeMemory();
}

/*
 * Dynamic server-synced Simulation
 */
forward CR_ProcessItems();
public	CR_ProcessItems()
{
	for(new i = 0; i <= CR_GetDynamicPool(); i++)
		CR_UpdateTransform(i);
}

/*
 * custom functions
 */

 // throws an object
 stock CR_ThrowObject(playerid, index, Float:speed)
 {
	if(CR_IsDynamicSlotUsed(index))
	{
		Streamer_Update(playerid);
		new Float:angle;
		GetPlayerFacingAngle(playerid, angle);
		CR_SetLinearVelocity(index, speed * floatsin(-angle, degrees), speed * floatcos(-angle, degrees), 1.7);
		return 1;
	}
	return 0;
 }

 // returns the information of the object in front of player, valid for vehicles as well
 stock CR_GetForwardObjectInfo(playerid, Float:distance, &modelid, &Float:boundingRadius, &isStatic)
 {
	new Float:x, Float:y, Float:z, Float:facing;
	GetPlayerPos(playerid, x, y, z);
	GetPlayerFacingAngle(playerid, facing);
	if(CR_RayCastObjectInfo(x, y, z, x + distance * floatsin(-facing, degrees), y + distance * floatcos(-facing, degrees), z, modelid, boundingRadius, isStatic))
		return 1;
	return 0;
 }

 // places an object on the ground correctly
 stock CR_PlaceObjectOnGround(objectid)
 {
	new Float:x, Float:y, Float:z, Float:yaw, Float:pitch, Float:roll;
	new modelid = Streamer_GetIntData(STREAMER_TYPE_OBJECT, objectid, E_STREAMER_MODEL_ID);
	new Float:minx, Float:miny, Float:minz, Float:maxx, Float:maxy, Float:maxz;
	GetDynamicObjectPos(objectid, x, y, z);
	GetDynamicObjectRot(objectid, roll, pitch, yaw);
	CR_GetAABB(modelid, x, y, z, yaw, pitch, roll, minx, miny, minz, maxx, maxy, maxz);
	new Float:dz = floatabs(z - minz);
	if(CR_RayCast(minx, miny, minz, minx, miny, -1000.0, maxx, maxy, maxz))
		return SetDynamicObjectPos(objectid, x, y, maxz + dz);
	return 0;
 }

 // passes the z coordinate for 2D coordinates by reference
 stock CR_FindZ_For2D(Float:x, Float:y, &Float:z)
 {
	if(CR_RayCast(x, y, 1000.0, x, y, -10000000.0, x, y, z))
		return 1;
	return 0;
 }

/*
 * forwarding CR_OnCollisionOccur. This callback gets called instantly as it detects collisions between any two objects
 * note that, the callback gets called too frequently, may be 10000 times per second
 */

 forward CR_OnCollisionOccur(modelid0, modelid1);
