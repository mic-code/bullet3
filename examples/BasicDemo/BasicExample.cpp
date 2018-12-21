/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"
#include "landscapeData.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}
	virtual void initPhysics();
	void createLargeMeshBody();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	//groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);


	/*btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -50, 0));*/

	/*{
		btScalar mass(0.);
		createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 1, 1));
	}*/

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = createBoxShape(btVector3(.1, .1, .1));

		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass, localInertia);

		for (int k = 0; k < ARRAY_SIZE_Y; k++)
		{
			for (int i = 0; i < ARRAY_SIZE_X; i++)
			{
				for (int j = 0; j < ARRAY_SIZE_Z; j++)
				{
					startTransform.setOrigin(btVector3(
						btScalar(0.2 * i),
						btScalar(2 + .2 * k),
						btScalar(0.2 * j)));

					createRigidBody(mass, startTransform, colShape);
				}
			}
		}
		createLargeMeshBody();
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}


static bool myCustomMaterialCombinerCallback(
	btManifoldPoint& cp,
	const btCollisionObjectWrapper* colObj0Wrap,
	int partId0,
	int index0,
	const btCollisionObjectWrapper* colObj1Wrap,
	int partId1,
	int index1)
{
	// one-sided triangles
	if (colObj1Wrap->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE)
	{
		auto triShape = static_cast<const btTriangleShape*>(colObj1Wrap->getCollisionShape());
		const btVector3* v = triShape->m_vertices1;
		btVector3 faceNormalLs = btCross(v[1] - v[0], v[2] - v[0]);
		faceNormalLs.normalize();
		btVector3 faceNormalWs = colObj1Wrap->getWorldTransform().getBasis() * faceNormalLs;
		btScalar nDotF = btDot(faceNormalWs, cp.m_normalWorldOnB);
		if (nDotF <= 0.0f)
		{
			// flip the contact normal to be aligned with the face normal
			cp.m_normalWorldOnB += -2.0 * nDotF * faceNormalWs;
		}
	}

	//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	return false;
}

void BasicExample::createLargeMeshBody()
{
	btTransform trans;
	trans.setIdentity();

	btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
	btIndexedMesh part;

	part.m_vertexBase = (const unsigned char*)LandscapeVtx;
	part.m_vertexStride = sizeof(btScalar) * 3;
	part.m_numVertices = 4;
	part.m_triangleIndexBase = (const unsigned char*)LandscapeIdx;
	part.m_triangleIndexStride = sizeof(short) * 3;
	part.m_numTriangles = 6/3;

	meshInterface->addIndexedMesh(part, PHY_SHORT);

	bool useQuantizedAabbCompression = true;
	btBvhTriangleMeshShape* trimeshShape = new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
	btVector3 localInertia(0, 0, 0);

	btRigidBody* body = createRigidBody(0, trans, trimeshShape);
	body->setFriction(btScalar(0.9));

	gContactAddedCallback = myCustomMaterialCombinerCallback;
}


void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
