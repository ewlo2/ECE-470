# -*- coding: utf-8 -*-
"""
Created on Sun Feb 16 18:59:10 2020

@author: Edmund Lo
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
import math
import matplotlib.pyplot as mpl
import time

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

#Retrieve joint handles
    
jointHandle = np.zeros((jointNum,),dtype=np.int)
for i in range(jointNum):
    returnCode, Handle = sim.simxGetObjectHandle(clientID,'UR3_joint'+str(i+1), sim.simx_opmode_blocking)
    if returnCode != sim.simx_return_ok:
        raise Exception('Could not get object handle for ' + str(i+1) + 'th  joint')
    jointHandle[i] = Handle
print('Joint Handles Retrieved') 
returnCode, forceSensorHandle = sim.simxGetObjectHandle(clientID,'UR3_connection', sim.simx_opmode_blocking)
print('Sensor Handle Retrieved') 
time.sleep(2)


# ==================================================================================================== #

# Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# ******************************** Your robot control code goes here  ******************************** #

time.sleep(1)

#Read Force Sensor (1st Time)
errorCode, forceState, forceVector, torqueVector = sim.simxReadForceSensor(clientID,forceSensorHandle,sim.simx_opmode_streaming)

#Set Joint Positions

targetPos1=[90*math.pi/180,90*math.pi/180,-90*math.pi/180,90*math.pi/180,-90*math.pi/180,90*math.pi/180]
for i in range(jointNum):
    returnCode = sim.simxSetJointTargetPosition(clientID, jointHandle[i], targetPos1[i],sim.simx_opmode_oneshot)
    time.sleep(0.5)

time.sleep(2)
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxGetPingTime(clientID)
sim.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

