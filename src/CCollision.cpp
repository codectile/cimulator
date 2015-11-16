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

#include "CStructures.h"
#include <fstream>
#include "btBulletDynamicsCommon.h"
#include "BulletCollision\CollisionDispatch\btGhostObject.h"
#include "CCollision.h"
#include <vector>

btCompoundShape* static_shapes;
btCompoundShape* dynamic_shapes;

std::vector<btCollisionObject*> cache_bodies;

ColData* cData;
cINST* inst;

void cr_readCadb()
{
	Header header;
	std::fstream file;
	file.open("scriptfiles\\colandreas\\colandreas.cadb", std::fstream::in | std::fstream::binary);
	if (file.is_open())
	{
		file.read((char*)&header, sizeof(Header));
		numCols = header.numColModels;
		numIPLs = header.numIPLs;
		cData = new ColData[header.numColModels];
		for (uint16_t i = 0; i < header.numColModels; i++)
		{
			file.read((char*)&cData[i], 8);
			if (cData[i].numSpheres > 0)
			{
				cData[i].sphere = new cSphere[cData[i].numSpheres];
				for (int s = 0; s < cData[i].numSpheres; s++)
				{
					file.read((char*)&cData[i].sphere[s].center, 12);
					file.read((char*)&cData[i].sphere[s].radius, 4);
				}
			}

			if (cData[i].numBoxes > 0)
			{
				cData[i].box = new cBox[cData[i].numBoxes];
				for (int b = 0; b < cData[i].numBoxes; b++)
				{
					file.read((char*)&cData[i].box[b].center, 12);
					file.read((char*)&cData[i].box[b].size, 12);
				}
			}

			if (cData[i].numFaces > 0)
			{
				cData[i].face = new cFaces[cData[i].numFaces];
				for (int f = 0; f < cData[i].numFaces; f++)
				{
					file.read((char*)&cData[i].face[f].face1, 12);
					file.read((char*)&cData[i].face[f].face2, 12);
					file.read((char*)&cData[i].face[f].face3, 12);
				}
			}
		}
		if (numIPLs > 0)
		{
			inst = new cINST[numIPLs];
			for (int i = 0; i < numIPLs; i++)
			{
				file.read((char*)&inst[i].modelid, 2);
				file.read((char*)&inst[i].pos, 12);
				file.read((char*)&inst[i].rot, 16);

			}
		}
		file.close();
	}
}

void cr_createColObject()
{
	static_shapes = new btCompoundShape[20000];
	dynamic_shapes = new btCompoundShape[20000];
	btQuaternion quat(0, 0, 0, 1);
	for (int i = 0; i < numCols; i++)
	{
		if (cData[i].numSpheres > 0)
		{
			for (int s = 0; s < cData[i].numSpheres; s++)
			{
				btSphereShape* sphere = new btSphereShape(cData[i].sphere[s].radius);
				static_shapes[cData[i].modelid].addChildShape(btTransform(quat, btVector3(cData[i].sphere[s].center.x, cData[i].sphere[s].center.y, cData[i].sphere[s].center.z)), sphere);
				dynamic_shapes[cData[i].modelid].addChildShape(btTransform(quat, btVector3(cData[i].sphere[s].center.x, cData[i].sphere[s].center.y, cData[i].sphere[s].center.z)), sphere);
			}
		}

		if (cData[i].numBoxes > 0)
		{
			for (int b = 0; b < cData[i].numBoxes; b++)
			{
				btBoxShape* box = new btBoxShape(btVector3((cData[i].box[b].size.x), (cData[i].box[b].size.y), (cData[i].box[b].size.z)));
				static_shapes[cData[i].modelid].addChildShape(btTransform(quat, btVector3(cData[i].box[b].center.x, cData[i].box[b].center.y, cData[i].box[b].center.z)), box);
				dynamic_shapes[cData[i].modelid].addChildShape(btTransform(quat, btVector3(cData[i].box[b].center.x, cData[i].box[b].center.y, cData[i].box[b].center.z)), box);
			}
		}

		if (cData[i].numFaces > 0)
		{
			btTriangleMesh* trimesh = new btTriangleMesh();
			btBvhTriangleMeshShape* mesh;
			btConvexTriangleMeshShape* convex;
			for (int f = 0; f < cData[i].numFaces; f++)
			{
				trimesh->addTriangle(btVector3(cData[i].face[f].face1.x, cData[i].face[f].face1.y, cData[i].face[f].face1.z),
					btVector3(cData[i].face[f].face2.x, cData[i].face[f].face2.y, cData[i].face[f].face2.z),
					btVector3(cData[i].face[f].face3.x, cData[i].face[f].face3.y, cData[i].face[f].face3.z));
			}
			mesh = new btBvhTriangleMeshShape(trimesh, true);
			convex = new btConvexTriangleMeshShape(trimesh);
			static_shapes[cData[i].modelid].addChildShape(btTransform(quat, btVector3(0, 0, 0)), mesh);
			dynamic_shapes[cData[i].modelid].addChildShape(btTransform(quat, btVector3(0, 0, 0)), convex);
		}
	}
	delete[] cData->sphere;
	delete[] cData->box;
	delete[] cData->face;
	delete[] cData;
}

void cr_objectPlacement(btDiscreteDynamicsWorld* dynamicsWorld, btScalar worldrest)
{
	for (int i = 0; i < numIPLs; i++)
	{
		/*
		btMotionState* motionState = new btDefaultMotionState(btTransform(btQuaternion(inst[i].rot.rx, inst[i].rot.ry, inst[i].rot.rz, inst[i].rot.rw), btVector3(inst[i].pos.x, inst[i].pos.y, inst[i].pos.z)));
		btRigidBody::btRigidBodyConstructionInfo information(0, motionState, (btCompoundShape*)&static_shapes[inst[i].modelid], btVector3(0, 0, 0));
		btRigidBody* rigidBody = new btRigidBody(information);
		rigidBody->setUserIndex(inst[i].modelid);
		rigidBody->setRestitution(worldrest);
		dynamicsWorld->addRigidBody(rigidBody);
		cache_bodies.push_back(rigidBody);
		
		* The above code is absolutely slow and it consumes a lot of memory. A better version of code is written below
		* which fast and efficient.
		*/
		btCollisionObject* colObj = new btCollisionObject();
		colObj->setCollisionShape((btCompoundShape*)&static_shapes[inst[i].modelid]);
		colObj->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT);
		colObj->setWorldTransform(btTransform(btTransform(btQuaternion(inst[i].rot.rx, inst[i].rot.ry, inst[i].rot.rz, inst[i].rot.rw), btVector3(inst[i].pos.x, inst[i].pos.y, inst[i].pos.z))));
		colObj->setUserIndex(inst[i].modelid);
		dynamicsWorld->addCollisionObject(colObj);
		cache_bodies.push_back(colObj);
	}
}

//all collision data related functions
btRigidBody* cr_addColBody(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btScalar mass, btVector3& position, btVector3& rotation, int inrt, int state)
{
	btQuaternion rot;
	rot.setEuler(rotation.getX(), rotation.getY(), rotation.getZ());
	btMotionState* motionState = new btDefaultMotionState(btTransform(rot, position));
	btVector3 inertia(0, 0, 0);
	if (inrt)
	{
		if (mass != 0.f)
			dynamic_shapes[modelid].calculateLocalInertia(mass, inertia);
	}
	btRigidBody::btRigidBodyConstructionInfo information(mass, motionState, (btCompoundShape*)(&dynamic_shapes[modelid]), inertia);
	btRigidBody* rigidBody = new btRigidBody(information);
	rigidBody->setActivationState(state);
	dynamicsWorld->addRigidBody(rigidBody);
	return rigidBody;
}

void cr_removeColBody(btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* rigidBody)
{
	dynamicsWorld->removeRigidBody(rigidBody);
}

void cr_deleteColBody(btDiscreteDynamicsWorld* dynamicsWorld, btRigidBody* rigidBody)
{
	//dynamicsWorld->removeRigidBody(rigidBody);
	delete rigidBody->getMotionState();
	delete rigidBody;
}

void cr_setCollisionShape(btRigidBody* rigidBody, int modelid)
{
	rigidBody->setCollisionShape((btCompoundShape*)&dynamic_shapes[modelid]);
}

void cr_getBoundingSphere(int modelid, btVector3& center, btScalar &radius)
{
	static_shapes[modelid].getBoundingSphere(center, radius);
}

void cr_getAABB(int modelid, btVector3& min, btVector3& max)
{
	btTransform transform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0));
	static_shapes[modelid].getAabb(transform, min, max);
}

int cr_isCompound(int modelid)
{
	if (static_shapes[modelid].isCompound())
		return 1;
	return 0;
}

int cr_getNumChildShapes(int modelid)
{
	return static_shapes[modelid].getNumChildShapes();
}

void cr_removeColMap(btDiscreteDynamicsWorld* dynamicsWorld)
{
	for (unsigned int i = 0; i < cache_bodies.size(); i++)
		dynamicsWorld->removeCollisionObject(cache_bodies[i]);
}

void cr_removeRoadBlocks(btDiscreteDynamicsWorld* dynamicsWorld)
{
	for (unsigned int i = 0; i < cache_bodies.size(); i++)
	{
		if (cache_bodies[i] != NULL && inst[i].modelid == 1662 || (inst[i].modelid >= 4504 && inst[i].modelid <= 4527)) //no default road blocks will be added
		{
			dynamicsWorld->removeCollisionObject(cache_bodies[i]);
			delete cache_bodies[i];
		}
	}
}

void cr_unLoad(btDiscreteDynamicsWorld* dynamicsWorld)
{
	if (!cache_bodies.empty())
	{
		for (unsigned int i = 0; i < cache_bodies.size(); i++)
		{
			if (cache_bodies[i] != NULL)
			{
				dynamicsWorld->removeCollisionObject(cache_bodies[i]);
				delete cache_bodies[i];
			}
		}
		cache_bodies.clear();
	}
}

//character collision approximations
btCollisionObject* cr_createCapsuleCharacter(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& position, btScalar radius, btScalar height)
{
	btCapsuleShape* capsuleApproximation = new btCapsuleShape(radius, height);
	btCollisionObject* character = new btCollisionObject();
	character->setCollisionFlags(character->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	character->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), position));
	character->setCollisionShape(capsuleApproximation);
	return character;
}


btGhostObject* cr_createGhost(btDiscreteDynamicsWorld* dynamicsWorld, int modelid, btVector3& position, btVector3& rotation)
{
	btGhostObject* ghost = new btGhostObject();
	ghost->setCollisionShape((btCompoundShape*)(&static_shapes[modelid]));
	ghost->setCollisionFlags(ghost->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
	btQuaternion quat;
	quat.setEuler(rotation.getX(), rotation.getY(), rotation.getZ());
	ghost->setWorldTransform(btTransform(quat, position));
	dynamicsWorld->addCollisionObject(ghost, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
	return ghost;
}

void cr_removeGhost(btDiscreteDynamicsWorld* dynamicsWorld, btGhostObject* ghost)
{
	dynamicsWorld->removeCollisionObject(ghost);
}

void cr_deleteGhost(btDiscreteDynamicsWorld* dynamicsWorld, btGhostObject* ghost)
{
	dynamicsWorld->removeCollisionObject(ghost);
	delete ghost->getCollisionShape();
	delete ghost;
}

int cr_getNumOverlappingObjects(btGhostObject* ghost)
{
	return ghost->getNumOverlappingObjects();
}
