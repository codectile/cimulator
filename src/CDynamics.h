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

#ifndef CDYNAMICS_H
#define CDYNAMICS_H

int cr_getNumCollisionObject(btDiscreteDynamicsWorld* dynamicsWorld);
void cr_setWorldGravity(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& axis);
btVector3 cr_getWorldGravity(btDiscreteDynamicsWorld* dynamicsWorld);
int cr_rayTrace(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& hit);
int cr_rayTraceEx(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& hit, int& modelid);
int cr_rayTraceNormal(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& normal);
int cr_rayTraceReflection(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, btVector3& reflection, btScalar& angle);
int cr_rayTraceInfo(btDiscreteDynamicsWorld* dynamicsWorld, btVector3& start, btVector3& end, int& modelid, float& boundRadius, int& isStatic);
void cr_setMass(btRigidBody* rigidBody, btScalar mass);
btScalar cr_getMass(btRigidBody* rigidBody);
void cr_getColTransform(btRigidBody* body, btVector3& pos, btVector3& rot);
void cr_simulate(btDiscreteDynamicsWorld* dynamicsWorld, int newtime, int oldtime);
void cr_setLinearVelocity(btRigidBody* rigidBody, btVector3& velocity);
void cr_setAngularVelocity(btRigidBody* rigidBody, btVector3& velocity);
btVector3 cr_getLinearVelocity(btRigidBody* rigidBody);
btVector3 cr_getAngularVelocity(btRigidBody* rigidBody);
void cr_setFriction(btRigidBody* rigidBody, btScalar friction);
btScalar cr_getFriction(btRigidBody* rigidBody);
void cr_setOrigin(btRigidBody* rigidBody, btVector3& pos);
btVector3 cr_getOrigin(btRigidBody* rigidBody);
void cr_setRotation(btRigidBody* rigidBody, btScalar yaw, btScalar pitch, btScalar roll);
void cr_getRotation(btRigidBody* rigidBody, btScalar &yaw, btScalar &pitch, btScalar &roll);
int cr_isActive(btRigidBody* rigidBody);
void cr_setRestitution(btRigidBody* rigidBody, btScalar rest);
btScalar cr_getRestitution(btRigidBody* rigidBody);
void cr_activate(btRigidBody* rigidBody);
void cr_deActivate(btRigidBody* rigidBody);
void cr_setTorque(btRigidBody* rigidBody, btVector3& torque);
void QuatToEuler(btQuaternion& rotation, btVector3& result);

#endif // !CDYNAMICS_H
