#include "ScissorSimulator.h"
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Primitives/Cylinder.h>
#include <Magnum/Primitives/UVSphere.h>
#include <Magnum/Math/Constants.h>
#include <Magnum/Math/Functions.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Transform.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <MagnumPlugins/AnySceneImporter/AnySceneImporter.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include "RigidBody.h"
#include <iostream>
using namespace Math::Literals;

#ifndef M_PI_2

#define M_PI_2 1.57079632679489661923

#endif

ScissorSimulator::ScissorSimulator()
{
	
	//Initialize the world*/
	_initDynamicWorld();
    //Camera setup	
	_setupCamera();
    //Drawing setup 
   _setupDrawings();

    //Bullet setup
	_bWorld->setGravity({0.0f, 0.0f, 0.0f});
	//_bWorld->getSolverInfo().m_globalCfm = 0.00001;
    _bWorld->setDebugDrawer(&_debugDraw);
	_bWorld->setInternalTickCallback(&ScissorSimulator::myTickCallBack,_bWorld);
	_bWorld->getSolverInfo().m_numIterations=160;
   //Create Objects in the scene
   _createSceneObjects();
	//Hinge contstaint setup
	_createHingeConstraint();
	//_setInitialRotation();
	_touch.initialize(true);

    //Movement Setup	
	//_movementController1.setMovingBody(_pivotPoint);
	//_movementController2.setMovingBody(_scissorRightTip);

}
void ScissorSimulator::startTimeline()
{
	 _timeline.start();	
}
void ScissorSimulator::_createSceneObjects()
{
	string meshPath = "C:/YashMagnum/Build/src/RelWithDebInfo/endowrist/";
	_loadRigidMesh(meshPath+"scissor_base.obj",_scissorBaseMesh,_scissorBase,1.0f);
	_loadRigidMesh(meshPath+"scissor_connector.obj",_scissorConnectorMesh,_scissorConnector,1.0f);
	_loadRigidMesh(meshPath+"scissor_left_tip.obj",_scissorLeftTipMesh,_scissorLeftTip,1.0f);
	_loadRigidMesh(meshPath+"scissor_right_tip.obj",_scissorRightTipMesh,_scissorRightTip, 1.0f);
	
	_scissorStick = new RigidBody{&_scene, 1.0f, (&_btHerStickShape), *_bWorld};
    new ColoredDrawable{*_scissorStick, _shader, _boxMesh, 0xFFFFFF_rgbf,
        Matrix4::scaling({2.0f, 2.0f, 40.0f}), _drawable};
	_scissorStick->setNoContactResponse();
	/*_sliderBase = new RigidBody{&_scene, 1.0f, (&_btBoxShape), *_bWorld};
    new ColoredDrawable{*_sliderBase, _shader, _boxMesh, 0xFFFFFF_rgbf,
        Matrix4::scaling({1.0f, 1.0f,1.0f}), _drawable};

	_sliderBase->setNoContactResponse();
	_base = new RigidBody{&_scene, 0.0f, (&_btBoxShape), *_bWorld};
    new ColoredDrawable{*_base, _shader, _boxMesh, 0x000000_rgbf,
        Matrix4::scaling({1.0f, 1.0f,1.0f}), _drawable};
	(_base->rigidBody()).getWorldTransform().setOrigin(origin);
	(_base->rigidBody()).getMotionState()->setWorldTransform((_base->rigidBody()).getWorldTransform());
    _base->setNoContactResponse();
	
	/*_pivotPoint = new RigidBody{&_scene, 1.0f, (&_btSphereShape), *_bWorld};
    new ColoredDrawable{*_pivotPoint, _shader, _sphereMesh, 0xFFFFFF_rgbf,
        Matrix4::scaling(Vector3{1.0f}), _drawable};
    _pivotPoint->setNoContactResponse();*/
}
void ScissorSimulator::_setupDrawings()
{
	_sphereMesh = MeshTools::compile(Primitives::uvSphereSolid(16, 32));
	_boxMesh = MeshTools::compile(Primitives::cubeSolid());
	
    _shader = Shaders::Phong{};
    _shader.setAmbientColor(0x111111_rgbf)
           .setSpecularColor(0x330000_rgbf)
           .setLightPosition({10.0f, 15.0f, 5.0f});
    _debugDraw = BulletIntegration::DebugDraw{};
    _debugDraw.setMode(BulletIntegration::DebugDraw::Mode::DrawWireframe);
	

    /* Setup the renderer so we can draw the debug lines on top */
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    GL::Renderer::enable(GL::Renderer::Feature::PolygonOffsetFill);
    GL::Renderer::setPolygonOffset(2.0f, 0.5f);
}
void ScissorSimulator::_setupCamera()
{
	(*(_cameraObject = new Object3D{&_scene}))
       .translate(Vector3{0.0f,3.0f,0.0f});
		

    (_camera = new SceneGraph::Camera3D(*_cameraObject))
        ->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
        .setProjectionMatrix(Matrix4::perspectiveProjection(90.0_degf, 1.0f, 0.001f, 100.0f))
        .setViewport(GL::defaultFramebuffer.viewport().size());
}
void ScissorSimulator::_initDynamicWorld()
{
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	_bBroadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
	//_bBroadphase = new btDbvtBroadphase();
	_bCollisionConfig = new btDefaultCollisionConfiguration();
	_bDispatcher = new btCollisionDispatcher(_bCollisionConfig);
	//_bSolver = new btSequentialImpulseConstraintSolver();
	btDantzigSolver* mlcp = new btDantzigSolver();
	_bSolver = new btMLCPSolver(mlcp);

	/* The world has to live longer than the scene because RigidBody
    instances have to remove themselves from it on destruction */
	_bWorld = new btDiscreteDynamicsWorld(_bDispatcher, _bBroadphase, _bSolver, _bCollisionConfig);	
}
void ScissorSimulator::_loadRigidMesh(std::string mesh_path,GL::Mesh& mesh, RigidBody* &rigidBody,btScalar mass) 
{
  PluginManager::Manager<Trade::AnySceneImporter> manager;
  Containers::Pointer<Trade::AnySceneImporter> importer =
      manager.loadAndInstantiate("AnySceneImporter");

  if(!importer)
	  cout << "No Importer" << endl;

  if (!importer || !importer->openFile(mesh_path))
  {
    cout << "Can't open "<<mesh_path<<" with AnySceneImporter" << endl;
	return;
  }
  Containers::Optional<Trade::MeshData3D> meshData = importer->mesh3D(0);
  Matrix4 rotationMatrix; rotationMatrix = rotationMatrix.rotation(-90.0_degf,Vector3{ 1,0,0 });
  MeshTools::transformPointsInPlace(rotationMatrix, (*meshData).positions(0));
  rotationMatrix = rotationMatrix.rotation(90.0_degf,Vector3{ 0,0,1 });
  MeshTools::transformPointsInPlace(rotationMatrix, (*meshData).positions(0));
  //Matrix4 scalingMatrix; scalingMatrix = scalingMatrix.scaling(Vector3{ 0.5,0.5,0.5 });
  //MeshTools::transformPointsInPlace(scalingMatrix, (*meshData).positions(0));
  mesh = MeshTools::compile(static_cast<Trade::MeshData3D &>(*meshData));
  std::vector<Vector3> positions = (*meshData).positions(0);
  rigidBody = new RigidBody{
      &_scene,mass, new btConvexHullShape{positions[0].data(), (int)positions.size(),sizeof(btScalar)*3}, *_bWorld};

 rigidBody->setNoContactResponse();
 (rigidBody->rigidBody()).setDamping(0.05, 0.85);
  new ColoredDrawable{*rigidBody, _shader, std::move(mesh), Color4::red(),
        Matrix4(), _drawable};
  return;
}
void ScissorSimulator::_createHingeConstraint()
{
	/*-----------------------*/
	/*Point2Point Constraints*/
	/*-----------------------*/
	/*btVector3 pivotInA3(0.0f,0.0f,-1.0f);
	btVector3 pivotInB3(0.0f,0.0f,1.0f);

	btPoint2PointConstraint* point2point = new btPoint2PointConstraint((_base->rigidBody()),(_sliderBase->rigidBody()),
																 pivotInA3,pivotInB3);
	//point2point->setParam(BT_P2P_FLAGS_CFM, 0.0f);
	//point2point->setParam(BT_P2P_FLAGS_ERP,0.8f);
	btJointFeedback* fbP2P = new btJointFeedback();
	point2point->setJointFeedback(fbP2P);
	_bWorld->addConstraint(point2point, true);*/
	
	/*------------------*/
	/*Slider Constraints*/
	/*------------------*/
	/*btTransform frameA0,frameB0;
	frameA0 = btTransform::getIdentity();
	frameB0 = btTransform::getIdentity();
	frameA0.setOrigin({0.0f,0.0f,0.0f});
	frameA0.getBasis().setEulerZYX(0,M_PI_2,0);
	frameB0.setOrigin({0.0f,0.0f,0.0f});
	frameB0.getBasis().setEulerZYX(0,M_PI_2,0);
	
	btSliderConstraint* slider = new btSliderConstraint( (_scissorStick->rigidBody()),(_sliderBase->rigidBody()),frameA0,frameB0,
																 true);
	slider->setParam(BT_CONSTRAINT_CFM, 1.0f);
	slider->setParam(BT_CONSTRAINT_ERP,0.8f);
	slider->setLowerLinLimit(-50);
	slider->setUpperLinLimit(50);
	slider->setLowerAngLimit(0);
	slider->setUpperAngLimit(0);
	btJointFeedback* fbSlider = new btJointFeedback();
	slider->setJointFeedback(fbSlider);
	_bWorld->addConstraint(slider, true);*/
	/*-----------------*/
	/*Fixed Constraints*/
	/*-----------------*/
	btTransform frameA, frameB;
	frameA = btTransform::getIdentity();
	frameB = btTransform::getIdentity();
	frameA.setOrigin(btVector3{0.0f,0.0f,8.86f});
	frameB.setOrigin(btVector3{0.0f,0.0f,-28.0f});
	
	btFixedConstraint* fixed = new btFixedConstraint( (_scissorBase->rigidBody()),(_scissorStick->rigidBody()),frameA,
																 frameB);
	fixed->setParam(BT_CONSTRAINT_CFM, 0.0f);
	fixed->setParam(BT_CONSTRAINT_ERP,0.8f);
	//p2p->setLowerLinLimit(-100);
	//p2p->setUpperLinLimit(100);
	btJointFeedback* fbFixed = new btJointFeedback();
	fixed->setJointFeedback(fbFixed);
	_bWorld->addConstraint(fixed, true);
	/*----------------*/
	/*Hinge Constraints*/
	/*----------------*/
	btVector3 pivotInA(0.0f,0.0f,-8.86f); 
	btVector3 pivotInB(0.0f,0.0f,-8.86);
	btVector3 axisInA(1.0f, 0.0f, 0.0f);
	btVector3 axisInB(1.0f, 0.0f, 0.0f);
	bool useReference = false;
	btHingeConstraint* hingeLeftRight = new btHingeConstraint((_scissorBase->rigidBody()), (_scissorConnector->rigidBody()),
																 pivotInA, pivotInB,
																 axisInA, axisInB,useReference);
	hingeLeftRight->setLimit(-1.57,1.57,0.0f);
	hingeLeftRight->setParam(BT_HINGE_FLAGS_CFM_STOP, 1.0f);
	hingeLeftRight->setParam(BT_HINGE_FLAGS_ERP_STOP,0.8f);
	btJointFeedback* fb = new btJointFeedback();
	hingeLeftRight->setJointFeedback(fb);
	_bWorld->addConstraint(hingeLeftRight, true);
	
	btTransform transA1,transB1;
	transA1.setIdentity();
	transB1.setIdentity();
	btVector3 originA1 = (_scissorConnector->rigidBody()).getWorldTransform().getOrigin() + btVector3{0.0f,0.0f,-17.34f};
	btVector3 originB1 = (_scissorLeftTip->rigidBody()).getWorldTransform().getOrigin() + btVector3{0.0f,0.0f,-17.34f};
	transA1.setOrigin(originA1);
	transB1.setOrigin(originB1);
	btGeneric6DofConstraint* hingeLeftTip = new btGeneric6DofConstraint((_scissorConnector->rigidBody()),(_scissorLeftTip->rigidBody()), 
																 transA1, transB1,
																 true);

	float cpm = 1.0;
	float erp = 0.8;
	float PI = 3.14f;
	hingeLeftTip->setLinearLowerLimit(btVector3(0, 0, 0));
	hingeLeftTip->setLinearUpperLimit(btVector3(0, 0, 0));
	hingeLeftTip->setAngularLowerLimit(btVector3(0,-PI,0)); 
	hingeLeftTip->setAngularUpperLimit(btVector3(0,PI,0));
	hingeLeftTip->setParam(BT_6DOF_FLAGS_CFM_STOP,cpm,0);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_CFM_STOP,cpm,1);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_CFM_STOP,cpm,2);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_CFM_STOP,cpm,3);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_CFM_STOP,cpm,4);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_CFM_STOP,cpm,5);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_ERP_STOP,erp,0);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_ERP_STOP,erp,1);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_ERP_STOP,erp,2);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_ERP_STOP,erp,3);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_ERP_STOP,erp,4);
	hingeLeftTip->setParam(BT_6DOF_FLAGS_ERP_STOP,erp,5);
	
	btJointFeedback* fb1 = new btJointFeedback();
	hingeLeftTip->setJointFeedback(fb1);

	_bWorld->addConstraint(hingeLeftTip, true);
	cout << hingeLeftTip->isLimited(0) << endl;
	cout << hingeLeftTip->isLimited(1) << endl;
	cout << hingeLeftTip->isLimited(2) << endl;
	cout << hingeLeftTip->isLimited(3) << endl;
	cout << hingeLeftTip->isLimited(4) << endl;
	cout << hingeLeftTip->isLimited(5) << endl;
	btTransform transA2,transB2;
	transA2.setIdentity();
	transB2.setIdentity();
	btVector3 originA2 = (_scissorConnector->rigidBody()).getWorldTransform().getOrigin()+ btVector3{0.0f,0.0f,-17.34f};
	btVector3 originB2 = (_scissorRightTip->rigidBody()).getWorldTransform().getOrigin() + btVector3{0.0f,0.0f,-17.34f};
	transA2.setOrigin(originA2);
	transB2.setOrigin(originB2);
	btGeneric6DofConstraint* hingeRightTip = new btGeneric6DofConstraint((_scissorConnector->rigidBody()),(_scissorRightTip->rigidBody()),
																 transA2, transB2,
																  true);
	//hingeRightTip->setLimit(-1.57,1.57,0.0f);
	hingeRightTip->setLinearLowerLimit(btVector3(0, 0, 0));
	hingeRightTip->setLinearUpperLimit(btVector3(0, 0, 0));
	hingeRightTip->setAngularLowerLimit(btVector3(0,-PI,0));
	hingeRightTip->setAngularUpperLimit(btVector3(0,PI,0));
	hingeRightTip->setParam(BT_6DOF_FLAGS_CFM_STOP ,cpm,0);
	hingeRightTip->setParam(BT_6DOF_FLAGS_CFM_STOP ,cpm,1);
	hingeRightTip->setParam(BT_6DOF_FLAGS_CFM_STOP ,cpm,2);
	hingeRightTip->setParam(BT_6DOF_FLAGS_CFM_STOP ,cpm,3);
	hingeRightTip->setParam(BT_6DOF_FLAGS_CFM_STOP ,cpm,4);
	hingeRightTip->setParam(BT_6DOF_FLAGS_CFM_STOP ,cpm,5);
	hingeRightTip->setParam(BT_6DOF_FLAGS_ERP_STOP ,erp,0);
	hingeRightTip->setParam(BT_6DOF_FLAGS_ERP_STOP ,erp,1);
	hingeRightTip->setParam(BT_6DOF_FLAGS_ERP_STOP ,erp,2);
	hingeRightTip->setParam(BT_6DOF_FLAGS_ERP_STOP ,erp,3);
	hingeRightTip->setParam(BT_6DOF_FLAGS_ERP_STOP ,erp,4);
	hingeRightTip->setParam(BT_6DOF_FLAGS_ERP_STOP ,erp,5);
	btJointFeedback* fb2 = new btJointFeedback();
	hingeRightTip->setJointFeedback(fb2);

	_bWorld->addConstraint(hingeRightTip, true); //Pass true as second argument to disable collision between conntected bodies.*/
	
}
/** callback from simulation at every tick. if number of manifold > 0 means there is a collision.
    set velocity of the cube to 0 so that it will not move backward*/
void ScissorSimulator::myTickCallBack(btDynamicsWorld *dynamicsWorld, btScalar timeStep)
{
	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	//std::cout << "Number of numManifolds = " << numManifolds << std::endl;
	btVector3 velocity{ 0.0f,0.0f,0.0f };
	for(int manifoldsIndex = 0; manifoldsIndex < numManifolds ; manifoldsIndex++ )	
	{
		//std::cout << "Collision Detected " << std::endl;		
		btPersistentManifold *contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(manifoldsIndex);
		btRigidBody* obA = const_cast<btRigidBody*>(btRigidBody::upcast((contactManifold->getBody1())));
		obA->setLinearVelocity(velocity);
		obA->setAngularVelocity(velocity);

		btRigidBody* obB = const_cast<btRigidBody*>(btRigidBody::upcast((contactManifold->getBody0())));
		obB->setLinearVelocity(velocity);
		obB->setAngularVelocity(velocity);
	}

	//Make velocities of rigidBody attached to constraints zero on every trick
	auto noOfConstraints = dynamicsWorld->getNumConstraints();
	//std::cout << "Number of Constraints = " << noOfConstraints << std::endl;
	for( auto index = 0; index < noOfConstraints; index++)
	{
		btTypedConstraint* cons = const_cast<btTypedConstraint*>(dynamicsWorld->getConstraint(index));
		btRigidBody& obA = cons->getRigidBodyA();
		obA.setLinearVelocity(velocity);
		obA.setAngularVelocity(velocity);

		btRigidBody& obB = cons->getRigidBodyB();
		obB.setLinearVelocity(velocity);
		obB.setAngularVelocity(velocity);
	}
}
/** draw the simulation */
void ScissorSimulator::drawEvent() {
	
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color|GL::FramebufferClear::Depth);

    /* Step bullet simulation */
    _bWorld->stepSimulation(_timeline.previousFrameDuration(), 1);
 //_bWorld->stepSimulation(0.016);   

    /* Draw the cubes */
    if(_drawCubes)
	{	
		_camera->draw(_drawable);

	}

    /* Debug draw. If drawing on top of cubes, avoid flickering by setting
       depth function to <= instead of just <. */
	if(_drawDebug) {
        if(_drawCubes)
            GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::LessOrEqual);
	    _debugDraw.setTransformationProjectionMatrix(
        _camera->projectionMatrix()*_camera->cameraMatrix());
		 _bWorld->debugDrawWorld();
		
        if(_drawCubes)
            GL::Renderer::setDepthFunction(GL::Renderer::DepthFunction::Less);
    }
	_touchPool(0);
}
void ScissorSimulator::timelineNextFrame()
{
	_timeline.nextFrame();
}


void ScissorSimulator::keyPressEvent(Sdl2Application::KeyEvent& event) {
	
    /* Key Event Handlers */
	switch(event.key())
	{
		case (Sdl2Application::KeyEvent::Key::D) :
		{
			if(_drawCubes )
				_drawCubes = false;
			else
				_drawCubes = true;
				
			if(_drawDebug)
				_drawDebug = false;
			else
				_drawDebug = true;
				
		}
		break;
		case (Sdl2Application::KeyEvent::Key::E) :
		{
			
			std::exit(0);
		}
		break;
		case (Sdl2Application::KeyEvent::Key::G) :
		{

		}
		break;
		case (Sdl2Application::KeyEvent::Key::Z) :
		{
			
		}
		break;
		case (Sdl2Application::KeyEvent::Key::X) :
		{
			
		}
		break;
		case (Sdl2Application::KeyEvent::Key::L) :
		{
			
		}
		break;
		case (Sdl2Application::KeyEvent::Key::R) :
		{
			
		}
		break;
		default:
		{
			_movementController1.keyPressEvent(event);
			_movementController2.keyPressEvent(event);
			//btVector3 pos = btVector3{0.0,0.0,0.0};
			//_updateToolPosition(pos);
		}
	}
    event.setAccepted();
}
/** handle view port event so that when user resize the window, simulation is scaled accordingly*/
void ScissorSimulator::viewportEvent(const Vector2i& size)
{
	GL::defaultFramebuffer.setViewport({{}, size});
	_camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
				.setViewport(size);
	
}
void ScissorSimulator::_updatePosition(RigidBody* rigidBody,btVector3 pos)
{
	(rigidBody->rigidBody()).getWorldTransform().setOrigin(pos);
	(rigidBody->rigidBody()).getMotionState()->setWorldTransform((rigidBody->rigidBody()).getWorldTransform());
}
void ScissorSimulator::_updateRelativeOrientation(RigidBody* rigidBody,btQuaternion quat)
{
	btMatrix3x3 orn = ((rigidBody->rigidBody()).getWorldTransform()).getBasis();
	btQuaternion origQuat;
	orn.getRotation(origQuat);
	orn.setRotation(quat * origQuat);
	(rigidBody->rigidBody()).getWorldTransform().setBasis(orn);
	(rigidBody->rigidBody()).getMotionState()->setWorldTransform((rigidBody->rigidBody()).getWorldTransform());
	
}
void ScissorSimulator::_updateAbsoluteOrientation(RigidBody* rigidBody,btQuaternion quat)
{

	btMatrix3x3 orn = ((rigidBody->rigidBody()).getWorldTransform()).getBasis();
	orn.setRotation(quat);
	(rigidBody->rigidBody()).getWorldTransform().setBasis(orn);
	(rigidBody->rigidBody()).getMotionState()->setWorldTransform((rigidBody->rigidBody()).getWorldTransform());
	
}
void ScissorSimulator::mousePressEvent(Sdl2Application::MouseEvent& event) {
	cout<<"Mouse Press Event" << endl;
	if ( event.button() == Sdl2Application::MouseEvent::Button::Left )
		_cameraObject->rotateY(10.0_degf);
	else 
		_cameraObject->rotateY(-10.0_degf);
	event.setAccepted();
}
void ScissorSimulator::mouseScrollEvent(Sdl2Application::MouseScrollEvent& event)
{
	Vector2 offset = event.offset();
	//cout << " X = " << offset.x() << " Y = " << offset.y() << endl;
	if( offset.y() > 0 )
	{
       _cameraObject->translateLocal(Vector3::zAxis(1.0f));
	}
	else if (offset.y() < 0 )
	{
		_cameraObject->translateLocal(Vector3::zAxis(-1.0f));
	}
	else
		cout << "No Mouse Scroll offset" << endl;
	
	event.setAccepted();
}
void ScissorSimulator::_touchPool(int tool_id)
{
	if( false == _touch.active())
		return;
	btVector3 _touchPosition = btVector3{float(_touch.position[tool_id].x),float(_touch.position[tool_id].y), float(_touch.position[tool_id].z)};
	//
	_touchPosition = (_touchPosition + btVector3{0.0f, 65.0f, 88.0f})/10.0f;

	btQuaternion _touchOrientation = btQuaternion{_touch.orientation[tool_id].x,_touch.orientation[tool_id].y,_touch.orientation[tool_id].z,_touch.orientation[tool_id].w};
	//_touchOrientation *= btQuaternion{0, 0, -0.3826834, 0.9238795};// Rotate 45 degree up to match the mesh position.

	//cout << "touchOrientation : x = " << _touchOrientation.x()<< " y = " << _touchOrientation.y() << " z = " << _touchOrientation.z() << " w = " << _touchOrientation.w()<<endl;
	_currentTouchPosition.touchPosition = _touchPosition;
	_currentTouchPosition.touchOrientation = _touchOrientation;

	_updateTool();
	_previousTouchPosition = _currentTouchPosition;
}
void ScissorSimulator::_updateTool()
{
	_updateAbsoluteOrientation(_scissorLeftTip,_currentTouchPosition.touchOrientation);
	_updatePosition(_scissorLeftTip,_currentTouchPosition.touchPosition);
	_updateAbsoluteOrientation(_scissorRightTip,_currentTouchPosition.touchOrientation);
	_updatePosition(_scissorRightTip,_currentTouchPosition.touchPosition);
}