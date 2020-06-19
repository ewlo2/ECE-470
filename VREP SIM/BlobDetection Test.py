# -*- coding: utf-8 -*-
"""
Created on Fri May 15 21:36:41 2020

@author: super
"""

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')
import sys
import numpy as np
import scipy as sp
from scipy import linalg as sl
import math
import matplotlib.pyplot as mpl
import time
import modern_robotics as mr

timeStep = 0.005
TIMEOUT = 5000

#Define joint parameters

jointNum = 6

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')

else:
    print ('Failed connecting to remote API server')
    sys.exit('Could not connect')

returnCode, camHandle = sim.simxGetObjectHandle(clientID,'cam', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('Could not get object handle for Camera')
print('Cam Handle Retrieved')

# ==================================================================================================== #

# Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

returnCode, packets = sim.simxGetStringSignal(clientID,'cords',sim.simx_opmode_streaming )
time.sleep(0.5)

returnCode, packets = sim.simxGetStringSignal(clientID,'cords',sim.simx_opmode_buffer )

packets = sim.simxUnpackFloats(packets)

ydist = 0.075
ptD = ydist/(0.3671875-0.306640625)
rotC = np.array([[1,0],
                 [0,-1]])

xCord = []
yCord = []

returnCode, camPos = sim.simxGetObjectPosition(clientID, camHandle, -1, sim.simx_opmode_blocking)
print(camPos)

for i in range(0,int(len(packets)),2):
    cord = np.array([packets[i], packets[i+1]])*ptD
    newCord = np.matmul(rotC,cord.T )
    xCord.append(newCord[0])
    yCord.append(newCord[1])

print(packets)

print(xCord)
print(yCord)


sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxGetPingTime(clientID)
sim.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")
