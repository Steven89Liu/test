import time
import matplotlib.pyplot as plt
import math
import sys
import numpy as np
import datetime
import os
#filename = 'AC_Transient_ZZ5_damp_N_1_m_8_Frequency_1.0Hz_DampingEta_0.4_09_04_2019_11_47.npz'
#filename = 'AC_Transient_ZZ5_damp_N_1_m_8_Frequency_1.0Hz_DampingEta_0.4_09_05_2019_09_28.npz'
filename = sys.argv[1]
#'AC_Transient_ZZ5_damp_N_1_m_8_Frequency_1.0Hz_DampingEta_0.4_09_05_2019_09_51.npz'
print('filename=', filename)
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
plt.show(block=False)
plt.savefig("Graph.png", format="PNG")
