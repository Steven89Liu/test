import pybullet as p

physicsClient = p.connect(p.GUI)
p.setGravity(0,0,-.10)

bunnyId = p.loadSoftBody("bunny.obj")


p.setRealTimeSimulation(1)

shift = [0,0,0]
meshScale=[0.12,0.12,0.12]
pos = [0,.05,0]
ori = [ 0.707,0,0, 0.707]


visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="head-tri.obj", rgbaColor=[1,1,1,1], 
                                    specularColor=[0.4,.4,0], visualFramePosition=shift, meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="head-tri.obj", 
                                          collisionFramePosition=shift,meshScale=meshScale
                                          )

p.createMultiBody(baseMass=0,baseInertialFramePosition=[0,0,0],baseCollisionShapeIndex=collisionShapeId, 
                   basePosition = pos, baseOrientation=ori)