import pybullet as p
import matplotlib.pyplot as plt
import time
import math
import numpy as np
import datetime
import os

def create1DMultiActuators(actuatorNumber,unitMotorNumber,
                           actuatorMass,actuatorLength,
                           actuatorWidth,actuatorThickness,
                           basePosition=[0,0,0],baseOrientation=[0,0,0,1]):

    N=actuatorNumber;
    m=unitMotorNumber;
    thickness=actuatorThickness;
    width=actuatorWidth;
    linkLength=actuatorLength/m;
    jointLength=0.5*linkLength;
    flagLength=0;
    flagMass=0.0;
    
    startBoxId = p.createCollisionShape(p.GEOM_BOX,
                                        halfExtents=[0.5*jointLength, 0.5*width, 0.5*thickness]);
    linkBoxId = p.createCollisionShape(p.GEOM_BOX,
                                       halfExtents=[0.5*linkLength, 0.5*width, 0.5*thickness],
                                       collisionFramePosition=[0.5*linkLength,0,0]);
    endBoxId = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[0.5*jointLength, 0.5*width, 0.5*thickness],
                                      collisionFramePosition=[0.5*jointLength,0,0]);

    flagBoxId = p.createCollisionShape(p.GEOM_BOX,
                                      halfExtents=[0.5*flagLength, 0.5*width, 0.5*thickness],
                                      collisionFramePosition=[0.5*flagLength,0,0]);

        
    mass = actuatorMass/(2*m);
    visualShapeId = -1;
    basePosition=basePosition;
    baseOrientation = baseOrientation;

    link_Masses = [actuatorMass/m for i in range(N*m-1)]
    link_Masses.append(actuatorMass/(2*m));
    link_Masses.append(flagMass);

    linkCollisionShapeIndices = [linkBoxId for i in range(N*m-1)]
    linkCollisionShapeIndices.append(endBoxId);
    linkCollisionShapeIndices.append(flagBoxId);

    linkVisualShapeIndices = [-1 for i in range(N*m+1)];

    linkPositions=[[0.5*jointLength, 0, 0]];
    for i in range(N*m-1):
        linkPositions.append([linkLength, 0, 0]);
    linkPositions.append([jointLength, 0, 0]);
    #linkPositions.append([linkLength, 0, 0]);


    linkOrientations = [[0, 0, 0, 1] for i in range(N*m+1)]

    linkInertialFramePositions = [[0.5*linkLength, 0, 0] for i in range(N*m-1)]
    linkInertialFramePositions.append([0.5*jointLength, 0, 0]);
    linkInertialFramePositions.append([0.5*flagLength, 0, 0]);

    linkInertialFrameOrientations = [[0, 0, 0, 1] for i in range(N*m+1)]
    indices = [i for i in range(N*m+1)]
    jointTypes = [p.JOINT_REVOLUTE for i in range(N*m)]
    jointTypes.append(p.JOINT_FIXED);
    axis = [[0, 1, 0] for i in range(N*m+1)]

    boxId = p.createMultiBody(mass,
                              startBoxId,
                              visualShapeId,
                              basePosition,
                              baseOrientation,
                              linkMasses=link_Masses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=indices,
                              linkJointTypes=jointTypes,
                              linkJointAxis=axis);

    jointNumber=p.getNumJoints(boxId)-1;
    #Disable the default motors
    for joint in range(jointNumber):
        p.setJointMotorControl2(boxId,
                                joint,
                                p.VELOCITY_CONTROL,
                                force=0, positionGain=1)
    return [boxId, jointNumber];


def generate1DMotorVoltages(actuatorVoltages,actuatorNumber,unitMotorNumber):
    motorVoltages=[];
    N=actuatorNumber;
    m=unitMotorNumber;
    for i in range(N):
        for j in range(m):
            motorVoltages.append(actuatorVoltages[i]/m);

    return motorVoltages;
actuator1DDensity=0.4806;
actuatorLength=10;
actuatorMass=actuator1DDensity*actuatorLength;

drivenFrequency=1.0; #in Hz
N=1;
m=5;
thickness=0.1;
width=0.3;
simTime=0;


timeStep=(5e-3)/240;
dampingEta=0.4;
simCycles=int(2.4e5);
recordStepInterval=5;


dataTime=[];
dataTheta=[];
dataAngularVelocities=[];
dataTor=[];
dataPositions=[];
dataMotorVoltages=[];


linkLength=actuatorLength/m;
jointLength=0.5*linkLength;

#p.connect(p.DIRECT)
p.connect(p.GUI)
p.resetSimulation()

def TorVolThe(Theta,angularVelocity,Voltage, join):
    thetaTarget=-1.5e-3*Voltage;
    if join==1:
        K=0.7796*1e5;
    if join==2:
        K=1.5735*1e5;
    if join==4:
        K=3.1630*1e5;
    else:
        K=1.5735*1e5*(0.5*join);
    omega=2*math.pi*drivenFrequency;
    #omega=1;
    Tor=-width*K*((Theta-thetaTarget)+(dampingEta/omega)*angularVelocity);
    #Tor=thetaTarget
    return Tor



(boxId,jointNumber)=create1DMultiActuators(N,m,actuatorMass,actuatorLength,width,thickness);

p.setTimeStep(timeStep);
p.setGravity(0, 0, 0)
p.changeDynamics(boxId, -1, spinningFriction=0.001, rollingFriction=0.001, 
                 linearDamping=0.0, angularDamping=0.0,jointDamping=0.0)

cid = p.createConstraint(boxId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

for step in range(simCycles):

    simTime=step*timeStep;

    theta=[];
    angularVelocities=[];
    positions=[];
    for joint in range(jointNumber):
        theta.append(p.getJointState(boxId,joint)[0]);
        angularVelocities.append(p.getJointState(boxId,joint)[1]);
        positions.append(p.getLinkState(boxId,joint)[4]);
    positions.append(p.getLinkState(boxId,jointNumber)[4]); #append the flag position

    actuatorVoltages=[200*math.cos(2*math.pi*drivenFrequency*simTime)];
    motorVoltages=generate1DMotorVoltages(actuatorVoltages, N, m);
    #print("jointNum=", jointNumber)
    if (step+1)%5000==0:
        print('N=',N,', m=',m,"Frequency=",round(drivenFrequency,2),"Hz, step=",step+1, "theta=", drivenFrequency*simTime/math.pi * 180)
        print("     joint[0] theta=", p.getJointState(boxId,0)[0], "joint[1] theta=", p.getJointState(boxId,1)[0], "joint[2] theta=", p.getJointState(boxId,2)[0],"joint[3] theta=", p.getJointState(boxId,3)[0], "joint[4] theta=", p.getJointState(boxId,4)[0])
        print("     thetaTargeti[0]=", -1.5e-3 *  motorVoltages[0], "thetaTargeti[1]=", -1.5e-3 *  motorVoltages[0], "thetaTargeti[2]=", -1.5e-3 *  motorVoltages[2]);

    Tor=[TorVolThe(theta[joint],angularVelocities[joint],motorVoltages[joint], jointNumber - joint) for joint in range(jointNumber)]


    if (step+1)%recordStepInterval==0 or step==0 or step==simCycles-1:
        dataTime.append(simTime);
        dataTheta.append(theta);
        dataAngularVelocities.append(angularVelocities);
        dataPositions.append(positions);
        dataMotorVoltages.append(motorVoltages);
        dataTor.append(Tor);
    #print("targetPosition=", Tor)
    #print("    theta=", theta)
    for joint in range(jointNumber):
        '''
        p.setJointMotorControl2(boxId,
                               joint,
                               p.POSITION_CONTROL,
                               targetPosition=Tor[joint],
                               force=1000000000000, positionGain=1)
        '''
        p.setJointMotorControl2(boxId,
                               joint,
                               p.TORQUE_CONTROL,
                               force=Tor[joint])
       
    p.stepSimulation();
    time.sleep(timeStep);

p.disconnect();
dataTime=np.array(dataTime);
dataTheta=np.array(dataTheta);
dataPositions=np.array(dataPositions);
dataTor=np.array(dataTor);
dateTime=datetime.datetime.today().strftime('%m_%d_%Y_%H_%M');
filename='AC_Transient_ZZ5_damp_N_'+str(N)+'_m_'+str(m)+'_Frequency_'+str(round(drivenFrequency,2))+'Hz_'+'DampingEta_'+str(round(dampingEta,2))+'_'+dateTime+'.npz';
outfile='data/'+filename;

print('outfile=', outfile)
np.savez(outfile,dataTime=dataTime,dataTheta=dataTheta,dataAngularVelocities=dataAngularVelocities,
         dataPositions=dataPositions,dataMotorVoltages=dataMotorVoltages,dataTor=dataTor, timeStep=timeStep, 
         simCycles=simCycles, recordStepInterval=recordStepInterval, N=N, m=m, drivenFrequency=drivenFrequency);
         
#''' 
loadFile=filename;
npzFile=np.load('data/'+loadFile);
N=npzFile['N'];
m=npzFile['m'];
dataTime=npzFile['dataTime'];
dataTheta=npzFile['dataTheta'];
dataPositions=npzFile['dataPositions'];
dataTor=npzFile['dataTor'];
dataMotorVoltages=npzFile['dataMotorVoltages'];
timeStep=npzFile['timeStep'];
simCycles=npzFile['simCycles'];
recordStepInterval=npzFile['recordStepInterval'];
plt.figure(figsize=(20,15))
for i in range(dataTheta.shape[1]):
    plt.plot(dataTime,-dataTheta[:,i],'o-',label='theta'+str(i+1)+'-data')
plt.xlabel("Time (s)")
plt.ylabel("Theta (rad)")
plt.title(loadFile+'_'+'theta-time')
plt.legend()
plt.show()
#'''
