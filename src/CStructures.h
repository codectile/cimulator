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

#ifndef CSTRUCTURES_S
#define CSTRUCTURES_S
#include <stdint.h>

struct TVector
{
	float x;
	float y;
	float z;
};
struct TQuaternion
{
	float rx;
	float ry;
	float rz;
	float rw;
};

struct Header
{
	char magic[4];
	uint16_t version;
	uint16_t numColModels;
	uint32_t numIPLs;
};

struct cSphere
{
	TVector center;
	float radius;
};

struct cBox
{
	TVector center;
	TVector size;
};

struct cFaces
{
	TVector face1;
	TVector face2;
	TVector face3;
};

struct ColData
{
	uint16_t modelid;
	uint16_t numSpheres;
	uint16_t numBoxes;
	uint16_t numFaces;
	cSphere* sphere;
	cBox* box;
	cFaces* face;
};

struct cINST
{
	uint16_t modelid;
	TVector pos;
	TQuaternion rot;
};


static int numCols;
static int numIPLs;
#endif // !CSTRUCTURES_S
