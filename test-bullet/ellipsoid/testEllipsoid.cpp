#include "btBulletCollisionCommon.h"
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

void addObjects(btCollisionWorld* world, vector<btCollisionShape*>& collisionShapes)
{
	btVector3 origin;
	btVector3 vx;
	btScalar rx, ry;

	btVector3 shapeOrigin(0.0, 0.0, 0.0);
	btScalar shapeRadius = 1.0;
	btVector3 x_old(8.0, 0.0, 0.0);
	btVector3 x_new;

	btVector3 axis;
	btScalar theta;
	btQuaternion rot;
	btTransform tr;
	btMultiSphereShape* shape;
	btCollisionObject* obj;
	
	//first object
	rx = 9.54750146076549;
	ry = 5.75396365699226;
	origin.setValue(13.8379985430549, -3.28213889380237, -28.3614926067146);
	vx.setValue(16.2790642615556, -6.74671275795975, -35.1466179627975);
	shape = new btMultiSphereShape(&shapeOrigin, &shapeRadius, 1);
	shape->setLocalScaling(btVector3(rx, ry, ry));
	shape->recalcLocalAabb();
	collisionShapes.push_back(shape);

	obj = new btCollisionObject;
	obj->setCollisionShape(shape);
	x_new = vx - origin;
	axis = btCross(x_old, x_new);
	theta = btAcos(btDot(x_old, x_new) / x_old.length() / x_new.length());
	rot = btQuaternion(axis, theta);
	tr.setOrigin(origin);
	tr.setRotation(rot);
	obj->setWorldTransform(tr);
	world->addCollisionObject(obj);

	//second object
	rx = 6.42339545069262;
	ry = 4.42800352622068;
	origin.setValue(25.9025144481244, - 4.9501503049078, - 32.6988102937523);
	vx.setValue(33.476868876405, - 2.97579191918069, - 31.0462171921409);
	shape = new btMultiSphereShape(&shapeOrigin, &shapeRadius, 1);
	shape->setLocalScaling(btVector3(rx, ry, ry));
	shape->recalcLocalAabb();
	collisionShapes.push_back(shape);

	obj = new btCollisionObject;
	obj->setCollisionShape(shape);
	x_new = vx - origin;
	axis = btCross(x_old, x_new);
	theta = btAcos(btDot(x_old, x_new) / x_old.length() / x_new.length());
	rot = btQuaternion(axis, theta);
	tr.setOrigin(origin);
	tr.setRotation(rot);
	obj->setWorldTransform(tr);
	world->addCollisionObject(obj);
}

struct MyCollisionResultCallback : public btCollisionWorld::ContactResultCallback
{
	virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
	{
		std::cout << colObj0Wrap->getCollisionObject()->getWorldArrayIndex() << "----" << colObj1Wrap->getCollisionObject()->getWorldArrayIndex() << "  " << cp.getDistance() << endl;
		return 1.0;
	}
};

int main()
{
	auto collisionConfiguration = new btDefaultCollisionConfiguration;
	auto dispatcher = new btCollisionDispatcher(collisionConfiguration);
	auto broadphase = new btDbvtBroadphase();
	auto world = new btCollisionWorld(dispatcher, broadphase, collisionConfiguration);
	vector<btCollisionShape*> collisionShapes;

	addObjects(world, collisionShapes);

	MyCollisionResultCallback callback;
	btCollisionObject* obA = world->getCollisionObjectArray()[0];
	btCollisionObject* obB = world->getCollisionObjectArray()[1];
	world->contactPairTest(obA, obB, callback);

	//clear 
	for (int i = world->getNumCollisionObjects() - 1; i >= 0; --i)
	{
		auto obj = world->getCollisionObjectArray()[i];
		world->removeCollisionObject(obj);
		delete obj;
	}
	for (int i = collisionShapes.size() - 1; i >= 0; --i)
	{
		btCollisionShape* shape = collisionShapes[i];
		collisionShapes[i] = 0;
		delete shape;
	}
	delete world;
	delete broadphase;
	delete dispatcher;
	delete collisionConfiguration;
}