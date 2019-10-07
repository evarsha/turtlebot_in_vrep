# ENPM 673 - Project 3 - Implementation of A* Algorithm on Turtlebot


###The 2 sample video files in the folder are generated to illustrate the path between following nodes:

1. [0,4]to[-4,-4].avi with Initial node as [0, 4] and Goal node as [-4, -4]

2. [-4,1]to[1,3].avi with Initial node as [-4,1] and Goal node as [1, 3]



###The libraries to be imported for executing the code:

1. import matplotlib.pyplot as plt

2. import math

3. import sys

4. import time


###The user defined parameters in the python file are:

1. Radius of the wheel of the turtlebot = 3.8cm

2. Length between the wheels = 23 cm

3. Time span for nodes generation = 2 second

4. Speed of left wheel and right wheel is given as [ rpm[0], rpm[1] ] for RPM_1 = 5 rad/s and RPM_2 = 10 rad/s  

5. Threshold for Goal node = 10 cm

6. Clearance for obstacle space = 60cm

7. Clearance for the robot in obstacle space in half plane equations = (17.7 + 60) = 77.7 cm

	


###Files need to be included in the same folder to have sucessful Remote API bindings

1. The files from V_Rep directory, "vrep.py" and "vrepConst.py" are in the same folder to run the python script.

2. "remoteApi.so" is for Linux, '.so' extension file is present in this folder , for other OS, other compatible files needed to be placed in this folder from (Download location) \V-REP_PRO_EDU_V3_6_0_Ubuntu16_04\programming\remoteApiBindings\lib\lib



###Steps to run the code:

1. The code is saved in the python script named " a*_algorithm_varsha_eranki "
	
2. Open V-REP and place the turtle-bot at the start location with initial orientation and initiate the simulation(the initial nodes, goal node and initial orientation should be given when the python file is run)

3. The Scatter plot shows the visited node exploration and the final path.
	






