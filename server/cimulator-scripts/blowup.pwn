#include <a_samp>
#include <streamer>
#include <cimulator>

new obj[10], vid;
new Float:vx, Float:vy, Float:vz;
main()
{
	print("\n----------------------------------");
	print("            blow up demo            ");
	print("----------------------------------\n");
}

new bullet; //stores object id of object to be shooted
public OnGameModeInit()
{
	CR_Load(); //loads the collision map of SanAndreas, don't use it if you don't want the map to be loaded
	CR_SetWorldGravity(0.0, 0.0, -3.5); //applies gravity on the Z axis
	SetGameModeText("blowup");
	AddPlayerClass(250,132.3336,-67.6250,1.5781,270.0000,0,0,0,0,0,0);
	CR_EnableSimulation();//enables the simulation
	obj[0] = CreateDynamicObject(1221, 135.68100, -91.73073, 1.06152,   0.00000, 0.00000, 0.00000);
	obj[1] = CreateDynamicObject(1221, 134.53696, -91.79777, 1.06152,   0.00000, 0.00000, 0.00000);
	obj[2] = CreateDynamicObject(1221, 133.09566, -91.88223, 1.06152,   0.00000, 0.00000, 0.00000);
	obj[3] = CreateDynamicObject(1221, 131.76138, -91.96041, 1.06152,   0.00000, 0.00000, 0.00000);
	obj[4] = CreateDynamicObject(1221, 134.45720, -91.80853, 3.18867,   0.00000, 0.00000, 0.00000);
	obj[5] = CreateDynamicObject(1221, 133.84770, -91.83816, 2.07358,   0.00000, 0.00000, 0.00000);
	obj[6] = CreateDynamicObject(1221, 132.42287, -91.92165, 2.07358,   0.00000, 0.00000, 0.00000);
	obj[7] = CreateDynamicObject(1221, 135.06685, -91.76672, 2.07358,   0.00000, 0.00000, 0.00000);
	obj[8] = CreateDynamicObject(1221, 133.13399, -91.89931, 3.18867,   0.00000, 0.00000, 0.00000);
	obj[9] = CreateDynamicObject(1221, 133.73849, -91.85670, 4.15000,   0.00000, 0.00000, 0.00000);

	//creating collision volumes to simulate ingame objects
	CR_CreateDynamicCol(0, obj[0], 1221, 1.5, 135.68100, -91.73073, 1.06152,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(1, obj[1], 1221,1.5, 134.53696, -91.79777, 1.06152,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(2, obj[2], 1221,1.5, 133.09566, -91.88223, 1.06152,   0.00000, 0.00000, 0.00000);

	CR_CreateDynamicCol(3, obj[3], 1221,1.5, 131.76138, -91.96041, 1.06152,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(4, obj[4], 1221,1.5, 134.45720, -91.80853, 3.18867,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(5, obj[5], 1221,1.5, 133.84770, -91.83816, 2.07358,   0.00000, 0.00000, 0.00000);

	CR_CreateDynamicCol(6, obj[6], 1221,1.5, 132.42287, -91.92165, 2.07358,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(7, obj[7], 1221,1.5, 135.06685, -91.76672, 2.07358,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(8, obj[8], 1221,1.5, 133.13399, -91.89931, 3.18867,   0.00000, 0.00000, 0.00000);
	CR_CreateDynamicCol(9, obj[9], 1221,1.5, 133.73849, -91.85670, 4.15000,   0.00000, 0.00000, 0.00000);
	return 1;
}

public OnGameModeExit()
{
	return 1;
}

public OnPlayerRequestClass(playerid, classid)
{
	return 1;
}

public OnPlayerConnect(playerid)
{
	SendClientMessage(playerid, -1, "cimulator scripts");
	return 1;
}

public OnPlayerDisconnect(playerid, reason)
{
	return 1;
}

public OnPlayerSpawn(playerid)
{
    	SetPlayerCameraPos(playerid, 132.3336,-67.6250,1.5781);
	SetPlayerCameraLookAt(playerid, 133.73849, -91.85670, 4.15000);
	return 1;
}

public OnPlayerDeath(playerid, killerid, reason)
{
	return 1;
}

public OnVehicleSpawn(vehicleid)
{
	return 1;
}

public OnVehicleDeath(vehicleid, killerid)
{
	return 1;
}

public OnPlayerText(playerid, text[])
{
	return 1;
}

public OnPlayerCommandText(playerid, cmdtext[])
{
    if (!strcmp("/shoot", cmdtext, true))
	{
	    if(!CR_IsDynamicIdxAvailable(10))
            CR_DestroyDynamicCol(10);
		new Float:x, Float:y, Float:z, Float:angle;
		GetPlayerPos(playerid, x, y, z);
		GetPlayerFacingAngle(playerid, angle);
		bullet = CreateDynamicObject(19342, x + 5 * floatsin(-angle, degrees), y + 5 * floatcos(-angle, degrees), z,   0.00000, 0.00000, 0.00000);
		CR_CreateDynamicCol(10, bullet, 19342, 1.5, x + 5 * floatsin(-angle, degrees), y + 5 * floatcos(-angle, degrees), z,   0.00000, 0.00000, 0.00000, 1);
		Streamer_Update(playerid);
 		CR_SetLinearVelocity(10, 15 * floatsin(-angle, degrees), 15 * floatcos(-angle, degrees), 1.7);
		return 1;
	}

	if (!strcmp("/reset", cmdtext, true))
	{
		CR_SetTransform(0, 135.68100, -91.73073, 1.06152,   0.00000, 0.00000, 0.00000);
		CR_SetTransform(1, 134.53696, -91.79777, 1.06152,   0.00000, 0.00000, 0.00000);
		CR_SetTransform(2, 133.09566, -91.88223, 1.06152,   0.00000, 0.00000, 0.00000);

		CR_SetTransform(3, 131.76138, -91.96041, 1.06152,   0.00000, 0.00000, 0.00000);
		CR_SetTransform(4, 134.45720, -91.80853, 3.18867,   0.00000, 0.00000, 0.00000);
		CR_SetTransform(5, 133.84770, -91.83816, 2.07358,   0.00000, 0.00000, 0.00000);

		CR_SetTransform(6, 132.42287, -91.92165, 2.07358,   0.00000, 0.00000, 0.00000);
		CR_SetTransform(7, 135.06685, -91.76672, 2.07358,   0.00000, 0.00000, 0.00000);
		CR_SetTransform(8, 133.13399, -91.89931, 3.18867,   0.00000, 0.00000, 0.00000);

		CR_SetTransform(9, 133.73849, -91.85670, 4.15000,   0.00000, 0.00000, 0.00000);
		return 1;
	}

	if (!strcmp("/destroy", cmdtext, true))
	{
		CR_DestroyAllColVolumes();
		return 1;
	}

	if (!strcmp("/stop", cmdtext, true))
	{
		CR_DisableSimulation();
		return 1;
	}

	if (!strcmp("/start", cmdtext, true))
	{
		CR_EnableSimulation();
		return 1;
	}
	
	if (!strcmp("/createvehicle", cmdtext, true))
	{
		GetPlayerPos(playerid, vx, vy, vz);
	 	vid = CreateVehicle(418, vx, vy, vz, 120.0, 255, 255, -1);
	 	PutPlayerInVehicle(playerid, vid, 0);
		return 1;
	}

	if (!strcmp("/createvcol", cmdtext, true))
	{
	 	new Float:rx, Float:ry, Float:rz, Float:rw;
	 	GetVehicleRotationQuat(vid, rw, rx, ry, rz);
	 	RemovePlayerFromVehicle(playerid);
  		CR_CreateVehicleCol(0, vid, 418, vx, vy, vz, rx, ry, rz, rw);
		return 1;
	}

	if (!strcmp("/removevehicle", cmdtext, true))
	{
        CR_RemoveVehicleCol(0);
		return 1;
	}
	return 0;
}

public OnPlayerEnterVehicle(playerid, vehicleid, ispassenger)
{
	return 1;
}

public OnPlayerExitVehicle(playerid, vehicleid)
{
	return 1;
}

public OnPlayerStateChange(playerid, newstate, oldstate)
{
	return 1;
}

public OnPlayerEnterCheckpoint(playerid)
{
	return 1;
}

public OnPlayerLeaveCheckpoint(playerid)
{
	return 1;
}

public OnPlayerEnterRaceCheckpoint(playerid)
{
	return 1;
}

public OnPlayerLeaveRaceCheckpoint(playerid)
{
	return 1;
}

public OnRconCommand(cmd[])
{
	return 1;
}

public OnPlayerRequestSpawn(playerid)
{
	return 1;
}

public OnObjectMoved(objectid)
{
	return 1;
}

public OnPlayerObjectMoved(playerid, objectid)
{
	return 1;
}

public OnPlayerPickUpPickup(playerid, pickupid)
{
	return 1;
}

public OnVehicleMod(playerid, vehicleid, componentid)
{
	return 1;
}

public OnVehiclePaintjob(playerid, vehicleid, paintjobid)
{
	return 1;
}

public OnVehicleRespray(playerid, vehicleid, color1, color2)
{
	return 1;
}

public OnPlayerSelectedMenuRow(playerid, row)
{
	return 1;
}

public OnPlayerExitedMenu(playerid)
{
	return 1;
}

public OnPlayerInteriorChange(playerid, newinteriorid, oldinteriorid)
{
	return 1;
}

public OnPlayerKeyStateChange(playerid, newkeys, oldkeys)
{
	return 1;
}

public OnRconLoginAttempt(ip[], password[], success)
{
	return 1;
}

public OnPlayerUpdate(playerid)
{
	return 1;
}

public OnPlayerStreamIn(playerid, forplayerid)
{
	return 1;
}

public OnPlayerStreamOut(playerid, forplayerid)
{
	return 1;
}

public OnVehicleStreamIn(vehicleid, forplayerid)
{
	return 1;
}

public OnVehicleStreamOut(vehicleid, forplayerid)
{
	return 1;
}

public OnDialogResponse(playerid, dialogid, response, listitem, inputtext[])
{
	return 1;
}

public OnPlayerClickPlayer(playerid, clickedplayerid, source)
{
	return 1;
}


/*
//uncomment to use
public CR_OnCollisionOccur(modelid0, modelid1)
{
	printf("\n%i\n%i", modelid0, modelid1);
	return 1;
}*/
