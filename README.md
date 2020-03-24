# ECE 470 SP20 - Introduction to Robotics 
Coppelia Simulation of UR3 Robot Arm with Python

## Project Update 1
Installed CoppeliaSim and Python 3.7
Set up remote API for Python by copying:

[*sim.py*](VREPSIM/sim.py)

[*simConst.py*](VREPSIM/simConst.py)

[*remoteApi.dll*](VREPSIM/remoteApi.dll)

from Coppelia lib folder to workspace where Coppelia Scene is located (named [Test1.py](VREP SIM/Test1.py)).

### Python Code Overview
Lines 7-21: Import necessary libraries

Lines 30-39: Closes any existing connections and establishes new connection between remote API server and the simulator

Lines 41-52: Retrieves Joint & Sensor Handles

Line 65: Reads Force Sensor state, force and torque values

Lines 69-72: Sets joint target positions according to target position array


### Resources
https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

https://www.youtube.com/watch?v=SQont-mTnfM

## Project Update 2
The deliverable required to demonstrate the application of forward kinematics to our Coppellia simulator for the UR3. As set up in the previous update, we constructed a Python script that would calculate the pose of the end effector of the UR3 robot given a set of joint angles.

The first task was to determine the orientation of the UR3 at its 'zero-configuration' and the distances between each joint center at that orientation. As mentioned in the ECE 470 Lab manual, it is prefered to retrieve the joint distances using simxGetObjectPostion instead of the dimensions given in the UR3 manual. 



### Code Overview

### Resources 
https://www.learnopencv.com/rotation-matrix-to-euler-angles/
