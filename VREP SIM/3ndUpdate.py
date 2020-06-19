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
import modern_robotics as mr


#Retrieving blob deectection coordinates
def get_blobCoord():
    returnCode, packets = sim.simxGetStringSignal(clientID,'cords',sim.simx_opmode_streaming )
    time.sleep(0.5)
    
    returnCode, packets = sim.simxGetStringSignal(clientID,'cords',sim.simx_opmode_buffer )
    
    packets = sim.simxUnpackFloats(packets)
    
    ydist = 0.075
    ptD = ydist/(0.3671875-0.306640625)
    print(ptD)
    rotC = np.array([[1,0],
                     [0,-1]])
    xCord = []
    yCord = []
    
    returnCode, camPos = sim.simxGetObjectPosition(clientID, camHandle, baseHandle, sim.simx_opmode_blocking)
    print(len(packets))
    
    for i in range(0,int(len(packets)),2):
        cord = np.array([packets[i], packets[i+1]])*ptD
        newCord = np.matmul(rotC,cord.T)
        xCord.append(newCord[0]+camPos[0])
        yCord.append(newCord[1]+camPos[1])
    
    return xCord, yCord

def normAng(angle):
    newAngle = angle;
    while (newAngle <= -math.pi):
        newAngle += 2*math.pi
    while (newAngle > math.pi):
        newAngle -= 2*math.pi
    return newAngle


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

#Returns Orientation of End Effector
def get_endAng():
    returnCode, eulerAngles = sim.simxGetObjectOrientation(clientID, endHandle, baseHandle, sim.simx_opmode_blocking)
    alpha = eulerAngles[0]
    beta = eulerAngles[1]
    gamma = eulerAngles[2]
    
    return alpha, beta, gamma
#

    
##Moves object to pose determined by Forward Kinematics
#def movePose(T, clientID, objectHandle):
#
#        print("Euler Angles:")
#        print(eulerAng)
#        print("position:")
#        print(P)
#        sim.simxSetObjectPosition(clientID,objectHandle, baseHandle, P, sim.simx_opmode_oneshot)
#        sim.simxSetObjectOrientation(clientID, objectHandle, baseHandle, eulerAng, sim.simx_opmode_oneshot)
    
#Returns the skew of a vector
def toSkew(x):
    skew = np.array([[0, -x[2][0], x[1][0]],
                    [x[2][0], 0, -x[0][0]],
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
                [1,0,0]])
    qAxis = np.array([[-2.23517418e-08, -7.45058060e-08,  1.04472965e-01],
                  [-1.11678056e-01, -3.15487705e-05,  1.08823866e-01],
                  [-1.11787505e-01,  2.95925856e-05,  3.52486223e-01],
                  [-1.11881070e-01, -1.48567764e-04,  5.65735817e-01],
                  [-1.12616174e-01, -1.85498764e-04,  6.49951100e-01],
                  [-1.11934401e-01, -1.88564445e-04,  6.51082993e-01]
                  ])
    S = []
    Sm = np.zeros((6,6))
    x,y,z = get_jointPos()
    
    for i in range(jointNum):
        w = np.array(rotAxis[i])
        
#        print(i)
#        print(w)
        q = np.array(qAxis[i])
#        q = np.array([x[i],y[i],z[i]])
#        print(q)
        
        
        v = -np.cross(w,q)
        #print(v)
        s = [w[0],w[1],w[2],v[0],v[1],v[2]]
#        print("s")
#        print(s)
        S.append(s)
    Sm = np.array([S[0],S[1],S[2],S[3],S[4],S[5]]).T
    return Sm

def moveJoints(targetPos):
    for i in range(jointNum):
        returnCode = sim.simxSetJointTargetPosition(clientID, jointHandle[i], targetPos[i],sim.simx_opmode_oneshot)
        time.sleep(.5) 

#Returns M, the matrix of body configuration in spatial frame when robot is in zero configuration
def zeroConfig(thetas):
    moveJoints(thetas)
    x, y, z = get_endPos()
    M = np.array([[0, 0, 1, x],
                  [0, 1, 0, y],
                  [-1, 0, 0, z],
                  [0, 0, 0, 1   ]])
    return M
        
def T(endPos,endAng):
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(endAng[0]), -math.sin(endAng[0])],
                   [0, math.sin(endAng[0]),  math.cos(endAng[0])]
                   ])
    
    Ry = np.array([[math.cos(endAng[1]), 0, math.sin(endAng[1])],
                   [0, 1, 0],
                   [-math.sin(endAng[1]), 0,  math.cos(endAng[1])]
                   ])
    
    Rz = np.array([[math.cos(endAng[2]), -math.sin(endAng[2]), 0 ],
                   [math.sin(endAng[2]), math.cos(endAng[2]), 0],
                   [0, 0, 1]
                   ])
    R = np.matmul(Rx,np.matmul(Ry,Rz))


    
    P = np.array([[endPos[0]],
                  [endPos[1]],
                  [endPos[2]]
                  ])
    T = np.vstack((np.c_[R,P],[0,0,0,1]))
    return T

def IKjointAngles(S, M, endPos, endAng, thetaList):
#    L1 = 0.152
#    L2 = 0.12
#    L3 = 0.244
#    L4 = 0.093
#    L5 = 0.213
#    L6 = 0.083
#    L7 = 0.083
#    L8 = 0.082
#    L9 = 0.0535
#    L10 = 0.059
    
#    xw = endPos[0]
#    yw = endPos[1]    
#    zw = endPos[2]  
#    wC = [xw, yw, zw]
#    yaw = 0
#    yaw1 = yaw*(math.pi/180)
#    
#    ad = [0.15, -0.15, -0.01]
#    gripC = [wC[0]+ad[0],wC[1]+ad[1],wC[2]+ad[2]]
#    cenC = [gripC[0]-L9*math.cos(yaw1), gripC[1]-L9*math.sin(yaw1), gripC[2]]
#    
#    
#    
#    theta1 = math.atan2(cenC[1], cenC[0]) - math.asin((0.027+L6)/math.sqrt(cenC[0]**2+cenC[1]**2))
#    
#    theta6 = math.pi/2 + theta1 -yaw1
#    endC = [cenC[0] + (math.sin(theta1-math.atan(L7/(0.027+L6)))*math.sqrt(L7**2+(0.027+L6)**2)), cenC[1] -(math.cos(theta1-math.atan(L7/(0.027+L6)))*math.sqrt(L7**2+(0.027+L6)**2)), cenC[2]+(L10+L8)]
#    
#    
#    d = endC[2]-L1
#    R = math.sqrt(endC[0]**2+endC[1]**2+ d**2)
#    a = math.asin(d/R)
#    b = math.acos((R**2 +L3**2 -L5**2)/(2*L3*R))
#    theta2 = -a-b
#    
#    gamma = math.acos((-R**2 +L3**2 + L5**2)/(2*L3*L5))
#    theta3 = math.pi-gamma
#
#    theta4 = a-(math.pi-b-gamma)
#    
#    
#    theta5 = -math.pi/2
    
    t = T(endPos, endAng)

    
    eomg = 0.01
    ev = 0.001  
    
    success = False
    thetalist, success = mr.IKinSpace(S, M, t, thetaList, eomg, ev)
    print(thetalist)
    print(success)
       
    thetaunRap = []
    #Normalizes theta in -2pi to 2pi range
    for theta in thetalist:
        thetaunRap.append(normAng(theta))
    
    print(thetaunRap)
    print(success)
    return thetaunRap
# ==================================================================================================== #
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
returnCode, endHandle = sim.simxGetObjectHandle(clientID,'suctionPadLink', sim.simx_opmode_blocking)

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

eggHandle = np.zeros((6,),dtype=np.int)
for i in range(6):
    returnCode, Handle = sim.simxGetObjectHandle(clientID,'Egg'+str(i), sim.simx_opmode_blocking)
    if returnCode != sim.simx_return_ok:
        raise Exception('Could not get object handle for ' + str(i) + 'th  egg')
    eggHandle[i] = Handle
print('Egg Handles Retrieved') 

#Retrieve reference object handle
returnCode, refHandle = sim.simxGetObjectHandle(clientID,'referenceObject', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
        raise Exception('Could not get object handle for Reference Object')
print('Reference Object Handle Retrieved')

returnCode, camHandle = sim.simxGetObjectHandle(clientID,'cam', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('Could not get object handle for Camera')
print('Cam Handle Retrieved')

returnCode, panHandle = sim.simxGetObjectHandle(clientID,'Pan', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('Could not get object handle for Pan')
print('Pan Handle Retrieved')

# ==================================================================================================== #

# Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)


#Inital Configuration at Zero Configuration
zero = [0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180]
moveJoints(zero)

time.sleep(5)
S = screw()
print("S")
print(S)
M = zeroConfig(zero)
print("M")
print(M)

#Move Arm to a set of joint angles
#jointAngle1 = [-40*math.pi/180,-50*math.pi/180,75*math.pi/180,-30*math.pi/180,-90*math.pi/180,0*math.pi/180]
#print(jointAngle1)
#moveJoints(jointAngle1)
#endPos = get_endPos()

#endPos = [0.024466101080179214, 0.11256171762943268, 0.5499975681304932]
#print("EndPos: ")
#print(endPos)
##endAng = get_endAng()
#endAng = [-0.0008262507035396993, -0.7834625244140625, 1.5705084800720215]
#print("EndAng: ")
#print(endAng)
#
#jointAngles = IKjointAngles(S, M, endPos,endAng)
#
#moveJoints(jointAngles)
#time.sleep(5)
#blobX, blobYY = get_blobCoord()

#returnCode, eggPosition = sim.simxGetObjectPosition(clientID, eggHandle[4], baseHandle, sim.simx_opmode_blocking)
#eggPosition[2] = eggPosition[2]+0.05
#
##returnCode, eggAng = sim.simxGetObjectOrientation(clientID, eggHandle[0], baseHandle, sim.simx_opmode_blocking)
#eggAng = [0,0,0]
#
#
#jointAngles = IKjointAngles(S, M, [eggPosition[0],eggPosition[1]+0.05,eggPosition[2]], eggAng, [0,0,0,0,0,0])
#moveJoints(jointAngles)
#
#jointAngles1 = IKjointAngles(S, M, eggPosition, eggAng, jointAngles)
#moveJoints(jointAngles1)
#
#
#jointAngles2 = IKjointAngles(S, M, [eggPosition[0],eggPosition[1],eggPosition[2]-0.025], eggAng , jointAngles1)
#moveJoints(jointAngles2)
#
#
#moveJoints(jointAngles)

#returnCode, panPosition = sim.simxGetObjectPosition(clientID, panHandle, baseHandle, sim.simx_opmode_blocking)
#panPosition[2] = panPosition[2]+0.1
#panPosition[0] = panPosition[0]+ 0.10
#panAng = [0,0,0]
#
#jointAnglesPan = IKjointAngles(S, M, panPosition, panAng, [0,0,0,0,0,0])
#moveJoints(jointAnglesPan)
#time.sleep(2)
#
#
#
#
#jointAnglesPan1 = IKjointAngles(S, M, [panPosition[0], panPosition[1], panPosition[2]], panAng, jointAnglesPan)
#moveJoints(jointAnglesPan1)
#time.sleep(2)
#
#moveJoints(jointAnglesPan)


sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
sim.simxGetPingTime(clientID)
sim.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")