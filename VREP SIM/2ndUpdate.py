# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 12:58:36 2020

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
import scipy as sp
from scipy import linalg as sl
import math
import matplotlib.pyplot as mpl
import time

#Function for retrieving joint center positions relative to base
#Returns lists of positions for all three axis
def get_jointPos():
    x, y, z =([] for i in range(3))
    for i in range(jointNum):
        returnCode, position = sim.simxGetObjectPosition(clientID, jointHandle[i], baseHandle, sim.simx_opmode_blocking)
        x.append(position[0])
        y.append(position[1])
        z.append(position[2])
    return x, y, z

#Returns Position of End Effector
    
def get_endPos():
    returnCode, position = sim.simxGetObjectPosition(clientID, endHandle, baseHandle, sim.simx_opmode_blocking)
    x = position[0]
    y = position[1]
    z = position[2]
    
    return x, y, z
#Returns the skew of a vector
def toSkew(x):
    skew = np.array([[0, -x[2][0], x[1][0]],
                    [x[2][0], 0[0], -x[0][0]],
                    [-x[1][0], x[0][0], 0]])   
    return skew

#Returns screw matrix of a screw axis vector
def screwToMatrix(x):
    matrix = np.array([[0, -x[2][0], x[1][0], x[3][0]],
                    [x[2][0], 0, -x[0][0], x[4][0]],
                    [-x[1][0], x[0][0], 0,x[5][0]],
                    [0, 0, 0, 0]])      
    return matrix

#Returns an array of screw axis of all joints 
def screw():
    
    rotAxis = np.array([[0,0,1],
                [-1,0,0],
                [-1,0,0],
                [-1,0,0],
                [0,0,1],
                [-1,0,0]])
    S = []
    x,y,z = get_jointPos()
    
    for i in range(jointNum):
        w = np.array([rotAxis[i]])
        #print(w)
        q = np.array([[x[i],y[i],z[i]]])
        #print(q)
        
        v = -np.cross(w,q)
        #print(v)
        S.append(np.array([[rotAxis[i][0],rotAxis[i][1],rotAxis[i][2],v[0][0],v[0][1],v[0][2]]]).reshape(-1,1))
    #print(S)
    return S

def moveJoints(targetPos):
    for i in range(jointNum):
        returnCode = sim.simxSetJointTargetPosition(clientID, jointHandle[i], targetPos[i],sim.simx_opmode_oneshot)
        time.sleep(0.5) 

#Returns M, the matrix of body configuration in spatial frame when robot is in zero configuration
def zeroConfig(thetas):
    moveJoints(thetas)
    x, y, z = get_endPos()
    M = np.array([[0, 0, -1, x],
                  [0, 1, 0, y],
                  [1, 0, 0, z],
                  [0, 0, 0, 1   ]])
    return M
#Returns T(theta) transformation for Forward Kinematics with screw axis array and joint angle (deg) inputs
def transformation(S,M,thetas):
    T = np.identity(4)
    
    for i in range(len(thetas)):
        print("expm" + str(i))
        print(screwToMatrix(S[i])*thetas[i])
        T = np.matmul(T,sl.expm(screwToMatrix(S[i])*thetas[i]))
        
        print("T" + str(i))
        print(T)
    T = np.matmul(T,M)
    return T

#Returns Euler Angles from rotation matrix
def rotToEuler(R):

    R11 = R[0][0]
    R21 = R[1][0]
    R22 = R[1][1]
    R23 = R[1][2]
    R31 = R[2][0]
    R32 = R[2][1]
    R33 = R[2][2]
    
    sy = math.sqrt(R11*R11 + R21*R21)
    
    
    
    if not (sy < 1e-6):
        xAngle = math.atan2(R32,R33)
        yAngle = math.atan2(-R31,sy)
        zAngle = math.atan2(R21, R11)
    else:
        xAngle = math.atan2(-R23,R22)
        yAngle = math.atan2(-R31, sy)
        zAngle = 0
                
    return np.array([xAngle,yAngle,zAngle])

#Moves object to pose determined by Forward Kinematics
def movePose(T, clientID, objectHandle):
        R = T[:3,:3]
        P = T[:3,3]
        
        eulerAng = rotToEuler(R)
        print("Euler Angles:")
        print(eulerAng)
        print("position:")
        print(P)
        sim.simxSetObjectPosition(clientID,objectHandle, -1, P, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(clientID, objectHandle, -1, eulerAng, sim.simx_opmode_oneshot)
        
        
        
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
    

#Retrieve base handle
    
returnCode, baseHandle = sim.simxGetObjectHandle(clientID,'UR3_link1_visible', sim.simx_opmode_blocking)

#Retrieve End Effector Handle
returnCode, endHandle = sim.simxGetObjectHandle(clientID,'UR3_link7_visible', sim.simx_opmode_blocking)

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

#Retrieve reference object handle
returnCode, refHandle = sim.simxGetObjectHandle(clientID,'referenceObject', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
        raise Exception('Could not get object handle for Reference Object')
print('Reference Object Handle Retrieved')

# ==================================================================================================== #

# Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
 


#Inital Postion
zero = [0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180]
time.sleep(1)
S = screw()
print(S)
M = zeroConfig(zero)
print(M)
T = transformation(S,M,zero)
print("T at zero")
print(T)

#Move reference frame to inital position
movePose(T,clientID,refHandle)
time.sleep(2)
#Move joints to position
for i in range(jointNum):
    returnCode = sim.simxSetJointTargetPosition(clientID, jointHandle[i], zero[i],sim.simx_opmode_oneshot)
    time.sleep(0.5)

    
#First Postion
targetPos1 = [90*math.pi/180,90*math.pi/180,-90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
T = transformation(S,M,targetPos1)


#Move reference frame to first position
movePose(T,clientID,refHandle)
time.sleep(2)

#Move joints to position
for i in range(jointNum):
    returnCode = sim.simxSetJointTargetPosition(clientID, jointHandle[i], targetPos1[i],sim.simx_opmode_oneshot)
    time.sleep(0.5)
time.sleep(10)

#Second Postion
targetPos2 = [-90*math.pi/180,45*math.pi/180,-90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180]
T = transformation(S,M,targetPos2)


#Move reference frame to first position
movePose(T,clientID,refHandle)
time.sleep(2)

#Move joints to position
for i in range(jointNum):
    returnCode = sim.simxSetJointTargetPosition(clientID, jointHandle[i], targetPos2[i],sim.simx_opmode_oneshot)
    time.sleep(0.5)
time.sleep(10)


sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxGetPingTime(clientID)
sim.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")