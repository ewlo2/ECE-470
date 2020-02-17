#ECE 470 SP19 - Introduction to Robotics 

##Project Update 1
Installed CoppeliaSim and Python 3.7
Set up remote API for Python by copying:
*sim.py
simConst.py
remoteApi.dll*
from Coppelia lib folder to workspace where Coppelia Scene is located (named Test1.py).
Python Code Overview
Lines 7-21: Import necessary libraries
Lines 30-39: Closes any existing connections and establishes new connection between remote API server and the simulator
Lines 41-52: Retrieves Joint & Sensor Handles
Line 65: Reads Force Sensor state, force and torque values
Lines 69-72: Sets joint target positions according to target position array


##Resources
https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
https://www.youtube.com/watch?v=SQont-mTnfM