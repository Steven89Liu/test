
#ifndef __SCISSOR_SIMULATOR_H__
#define __SCISSOR_SIMULATOR_H__

#include <btBulletDynamicsCommon.h>
#include <Magnum/Timeline.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Shaders/Phong.h>
#include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/GL/Renderer.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include "RigidBody.h"
#include "ColoredDrawable.h"
#include "MovementController.h"
#include "Simulation.h"
#include "touch.h"
/*#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif*/

using namespace Magnum;
using namespace Magnum::Platform;
using namespace std;
using namespace scind;
typedef SceneGraph::Object<SceneGraph::MatrixTransformation3D> Object3D;
typedef SceneGraph::Scene<SceneGraph::MatrixTransformation3D> Scene3D;


class ScissorSimulator : public Simulation{
		public:
			explicit ScissorSimulator();
			~ScissorSimulator(){}
			static void myTickCallBack(btDynamicsWorld *dynamicsWorld, btScalar timeStep);
			void drawEvent();
			void keyPressEvent(Sdl2Application::KeyEvent& event);
			void mousePressEvent(Sdl2Application::MouseEvent& event);
			void mouseScrollEvent(Sdl2Application::MouseScrollEvent& event);
			void viewportEvent(const Vector2i& size);
			void startTimeline();
			void timelineNextFrame();
		private :

			void _initDynamicWorld();
			void _setupCamera();
			void _setupDrawings();
			void _createSceneObjects();
			void _createHingeConstraint();
			void _updateAbsoluteOrientation(RigidBody* rigidBody,btQuaternion quat);
			void _updateRelativeOrientation(RigidBody* rigidBody,btQuaternion quat);
			void _updatePosition(RigidBody* rigidBody,btVector3 pos);
			void _touchPool(int tool_id);
			void _loadRigidMesh(std::string mesh_path, GL::Mesh& mesh, RigidBody* &rigidBody, btScalar mass);
						
			GL::Mesh _sphereMesh{NoCreate},_boxMesh{NoCreate};
			GL::Mesh _scissorBaseMesh{NoCreate},_scissorConnectorMesh{NoCreate}, _scissorLeftTipMesh{NoCreate},_scissorRightTipMesh{NoCreate};
			Shaders::Phong _shader{NoCreate};
			BulletIntegration::DebugDraw _debugDraw{NoCreate};

			//btDbvtBroadphase* _bBroadphase;
			btBroadphaseInterface* _bBroadphase;
			btDefaultCollisionConfiguration* _bCollisionConfig;
			btCollisionDispatcher* _bDispatcher;//{&_bCollisionConfig};
			//btSequentialImpulseConstraintSolver* _bSolver;
			btMLCPSolver* _bSolver;

			/* The world has to live longer than the scene because RigidBody
           instances have to remove themselves from it on destruction */
			btDiscreteDynamicsWorld* _bWorld;//{&_bDispatcher, &_bBroadphase, &_bSolver, &_bCollisionConfig};

			Scene3D _scene;
			SceneGraph::Camera3D* _camera;
			SceneGraph::DrawableGroup3D _drawable;
			Timeline _timeline;

			Object3D *_cameraObject;
		
			btBoxShape _btHerStickShape{{1.0f,1.0f,40.0f}},_btBoxShape{{1.0,1.0,1.0}};
			//btBoxShape _btHingeShape{{0.20f,0.20f,1.0f}};
			//btBoxShape _btBaseStickShape{{0.20f,0.20f,10.0f}};
			btSphereShape _btSphereShape{0.25f};
			//btSphereShape _btTargetSphereShape{3.0f};
			bool _drawCubes{true}, _drawDebug{false};

			RigidBody* _scissorBase;
			RigidBody* _scissorConnector;
			RigidBody* _scissorLeftTip;
			RigidBody* _scissorRightTip;
			RigidBody* _scissorStick;
			//RigidBody* _sliderBase;
			//RigidBody* _base;
			//RigidBody* _pivotPoint;
			
			
			Touch _touch;
			Vector3 _touchPosition;
			Vector4 _touchOrientation;
			MovementController _movementController1;
			MovementController _movementController2;
			btVector3 origin{btVector3{0.0f,0.0f,10.0f}};
			/*btHingeConstraint* hingeLeftTip;
			btHingeConstraint* hingeRightTip;
			btHingeConstraint* hingeLeftRight;
			btHingeConstraint* hingeTopBottom;*/
			//btPoint2PointConstraint* p2p;
			typedef struct 
					{
						btVector3 touchPosition{0.0,0.0,0.0};
						btQuaternion touchOrientation{0.0,0.0,0.0,0.0};
					}TouchPositionAndOrientation;
			TouchPositionAndOrientation _currentTouchPosition;
			TouchPositionAndOrientation _previousTouchPosition;
			void _updateTool();
};
#endif