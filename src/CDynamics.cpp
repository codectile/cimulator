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

#include "btBulletDynamicsCommon.h"
#define SIMD_DEG_TO_RAD	 btScalar(0.0174532925)
#define SIMD_RADIAN_TO_DEG	 btScalar(57.29577951)

//dynamics world
int cr_getNumCollisionObject(btDiscreteDynamicsWorld* dynamicsWorld)
{
	return dynamicsWorld->getNumCollisionObjects();
}

void cr_setWorldGravity(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& axis)
{
	dynamicsWorld->setGravity(axis);
}

btVector3 cr_getWorldGravity(btDiscreteDynamicsWorld* dynamicsWorld)
{
	return dynamicsWorld->getGravity();
}

int cr_rayTrace(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& hit)
{
	btCollisionWorld::ClosestRayResultCallback onHit(start, end);
	dynamicsWorld->rayTest(start, end, onHit);
	if (onHit.hasHit())
	{
		hit = onHit.m_hitPointWorld;
		return 1;
	}
	return 0;
}

int cr_rayTraceEx(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& hit, int& modelid)
{
	btCollisionWorld::ClosestRayResultCallback onHit(start, end);
	dynamicsWorld->rayTest(start, end, onHit);
	if (onHit.hasHit())
	{
		hit = onHit.m_hitPointWorld;
		modelid = onHit.m_collisionObject->getUserIndex();
		return 1;
	}
	return 0;
}

int cr_rayTraceNormal(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& normal)
{
	btCollisionWorld::ClosestRayResultCallback onHit(start, end);
	dynamicsWorld->rayTest(start, end, onHit);
	if (onHit.hasHit())
	{
		normal = onHit.m_hitNormalWorld;
		return 1;
	}
	return 0;
}

int cr_rayTraceReflection(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& reflection, btScalar& angle)
{
	btCollisionWorld::ClosestRayResultCallback onHit(start, end);
	dynamicsWorld->rayTest(start, end, onHit);
	if (onHit.hasHit())
	{
		btVector3 i = (onHit.m_hitPointWorld - start);
		reflection = (-2.f * i.dot(onHit.m_hitPointWorld.normalize()) * onHit.m_hitPointWorld.normalize()) + i;
		reflection = reflection.normalize();
		angle = reflection.angle(onHit.m_hitPointWorld.normalize()) * SIMD_RADIAN_TO_DEG;
		return 1;
	}
	return 0;
}

void cr_simulate(btDiscreteDynamicsWorld* dynamicsWorld, int newtime, int oldtime)
{
	dynamicsWorld->stepSimulation((((btScalar)(newtime - oldtime)) / 1000.f), 30);
}

//rigid body
void cr_setMass(btRigidBody* rigidBody, btScalar mass)
{
	if (mass != 0.f)
	{
		btVector3 inertia;
		rigidBody->getCollisionShape()->calculateLocalInertia(mass, inertia);
		rigidBody->setMassProps(mass, inertia);
	}
}

btScalar cr_getMass(btRigidBody* rigidBody)
{
	btScalar mass = btScalar(1) / (rigidBody->getInvMass());
	return mass;
}

int cr_isActive(btRigidBody* rigidBody)
{
	if (rigidBody->isActive())
		return 1;
	return 0;
}

void cr_getColTransform(btRigidBody* body, btVector3& pos, btVector3& rot)
{
	btTransform transform;
	
	body->getMotionState()->getWorldTransform(transform);
	pos = transform.getOrigin();

	btScalar yaw, pitch, roll;
	transform.getBasis().getEulerZYX(yaw, pitch, roll);
	rot.setX(roll * SIMD_RADIAN_TO_DEG);
	rot.setY(pitch * SIMD_RADIAN_TO_DEG);
	rot.setZ(-yaw * SIMD_RADIAN_TO_DEG);
	
}

void cr_setOrigin(btRigidBody* rigidBody, btVector3& pos)
{
	btTransform transform;
	transform.setIdentity();
	rigidBody->getMotionState()->getWorldTransform(transform);
	transform.setOrigin(pos);
	rigidBody->getMotionState()->setWorldTransform(transform);
	rigidBody->setWorldTransform(transform);
}

btVector3 cr_getOrigin(btRigidBody* rigidBody)
{
	btTransform transform;
	rigidBody->getMotionState()->getWorldTransform(transform);
	return transform.getOrigin();
}

void cr_setRotation(btRigidBody* rigidBody, btScalar yaw, btScalar pitch, btScalar roll)
{
	btTransform transform;
	btQuaternion quat;
	quat.setEulerZYX(-yaw * SIMD_DEG_TO_RAD, pitch * SIMD_DEG_TO_RAD, roll * SIMD_DEG_TO_RAD);
	rigidBody->getMotionState()->getWorldTransform(transform);
	transform.setRotation(quat);
	rigidBody->getMotionState()->setWorldTransform(transform);
}

void cr_getRotation(btRigidBody* rigidBody, btScalar &yaw, btScalar &pitch, btScalar &roll)
{
	btTransform transform;
	rigidBody->getMotionState()->getWorldTransform(transform);
	transform.getBasis().getEulerZYX(yaw, pitch, roll);
}

void cr_setLinearVelocity(btRigidBody* rigidBody, btVector3& velocity)
{
	rigidBody->setLinearVelocity(velocity);
}

void cr_setAngularVelocity(btRigidBody* rigidBody, btVector3& velocity)
{
	rigidBody->setAngularVelocity(velocity);
}

btVector3 cr_getLinearVelocity(btRigidBody* rigidBody)
{
	return rigidBody->getLinearVelocity();
}

btVector3 cr_getAngularVelocity(btRigidBody* rigidBody)
{
	return rigidBody->getAngularVelocity();
}

void cr_setFriction(btRigidBody* rigidBody, btScalar friction)
{
	rigidBody->setFriction(friction);
}

btScalar cr_getFriction(btRigidBody* rigidBody)
{
	return rigidBody->getFriction();
}

void cr_setRestitution(btRigidBody* rigidBody, btScalar rest)
{
	rigidBody->setRestitution(rest);
}

btScalar cr_getRestitution(btRigidBody* rigidBody)
{
	return rigidBody->getRestitution();
}

void cr_setTorque(btRigidBody* rigidBody, btVector3& torque)
{
	rigidBody->applyTorque(torque);
}

void cr_activate(btRigidBody* rigidBody)
{
	rigidBody->activate(true);
}

void cr_deActivate(btRigidBody* rigidBody)
{
	//rigidBody->setActivationState(DISABLE_SIMULATION);
	rigidBody->forceActivationState(DISABLE_SIMULATION);
}
