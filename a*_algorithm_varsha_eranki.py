#VARSHA ERANKI
#UID 116204569
#A Star ALGORITHM FOR TURTLRBOT


# importing the libraries

import math
import matplotlib.pyplot as plt
import sys

# defining the diffrential constraints

def diff_constraint(node,rpm,theta):
    
    new_node=[0,0]
    r = 0.038
    L = 0.23
    dt = 2/100
    x = node[0]
    y = node[1]
    ul = rpm[0]
    ur = rpm[1]
    
    for i in range(100):
        x_dot = (r*math.cos(theta)*(ul+ur))/2
        y_dot = (r*math.sin(theta)*(ul+ur))/2
        theta_dot = (r*(ur-ul))/L
    
        dx = x_dot*dt
        dy = y_dot*dt
        dtheta = theta_dot*dt
    
        x = x + dx
        y = y + dy
        theta = theta + dtheta
        
    x = ((math.floor(x * 100)) / 100.0)
    y = ((math.floor(y * 100)) / 100.0)
    
    new_node[0] = new_node[0]+x
    new_node[1] = new_node[1]+y
    # print("\n",node, round(theta,4), rpm)    
    
    return new_node[0], new_node[1], round(theta,4),rpm
    

# Obstacle space with origin at the center of the workspace

def workspace(x,y):
    #x = node[0]
    #y = node[1]
    o = 0
    clearance = 0.6
    d = 0.177 + clearance
    
    # boundary condition
    
    if x<-5.55+d or x >5.55-d or y<-5.05+d or y>5.05-d:
        o=1
        
    # Circular table
    
    elif ((x+4.05)**2 + (y-3.25)**2 ) <= (0.7995 + d)**2 or ((x+2.46)**2 + (y-3.25)**2 ) <= (0.7995 + d)**2:
        o=1
    elif -4.05 - d <= x and x <= -2.45 + d and 2.45 - d <= y and y <= 4.05 + d:
        o=1
        
    # Circle
    
    elif ((x+1.17)**2 + (y-2.31)**2 ) <= (0.405 + d)**2 :
        o=1
    elif ((x+1.65)**2 + (y-4.6)**2 ) <= (0.405 + d)**2 :
        o=1
    elif ((x+1.6)**2 + (y+4.6)**2 ) <= (0.405 + d)**2 :
        o=1
    elif ((x+1.17)**2 + (y+2.31)**2 ) <= (0.405 + d)**2 :
        o=1
    
        
    # Bottom right Long Rectangle touching x-axis 
    
    elif 1.3 - d <= x and x <= 5.55 + d and -5.05 - d <= y and y <= -4.7 + d:       
        o=1
    
    elif -0.81 - d <= x and x <= 1.93 + d and -4.7 - d <= y and y <= -3.18 + d:        
        o=1

    elif 2.24 - d <= x and x <= 3.41 + d and -4.7 - d <= y and y <= -4.12 + d:        
        o=1

    elif 3.72 - d <= x and x <= 5.55 + d and -4.7 - d <= y and y <= -3.94 + d:        
        o=1

    # Other Rectangles
    
    elif -1.17 - d <= x and x <= -0.26 + d and -1.9 - d <= y and y <= -0.08 + d:        
        o=1
        
    elif -0.26 - d <= x and x <= 1.57 + d and -2.4 - d <= y and y <= -1.64 + d:        
        o=1
        
    elif 4.97 - d <= x and x <= 5.55 + d and -3.27 - d <= y and y <= -2.1 + d:        
        o=1
        
    elif 4.64 - d <= x and x <= 5.55 + d and -1.42 - d <= y and y <= -0.57 + d:        
        o=1
    
    elif 2.29 - d <= x and x <= 3.8 + d and -2.38 - d <= y and y <= -1.21 + d:        
        o=1
        
    elif 2.77 - d <= x and x <= 3.63 + d and 3.22 - d <= y and y <= 5.05 + d:        
        o=1
        
    elif 4.28 - d <= x and x <= 4.71 + d and 4.14 - d <= y and y <= 5.05 + d:        
        o=1
        
    elif 4.97 - d <= x and x <= 5.55 + d and -0.57 - d <= y and y <= 0.6 + d:        
        o=1
        
    elif 1.89 - d <= x and x <= 5.55 + d and 1.16 - d <= y and y <= 1.92 + d:        
        o=1  
    
    return o


#print(workspace(1,3))


# Cost to come

def cost_to_come(current_node,parent_node):
    c = math.sqrt ( (current_node[0] - parent_node[0])**2 +  (current_node[1] - parent_node[1])**2 )
    return c
    

# heuristic cost

def heu(node):
    h = math.sqrt ( (node[0] - goal[0])**2 +(node[1] - goal[1])**2 )
    return h


def append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list):
    
    flag=0
    for cku in range(0,len(cost)):
        if(child_node == child_list[cku]):
            flag=1
            # print("child_node == child_list[cku]")
            if(cost[cku]>=(cost[x]+child_cost)):
                
                parent_list[cku]=parent_node
            
                cost[cku]=round((cost[x]+child_cost),4)
                theta_list[cku] = child_theta
                rpm_list[cku] = rpm
                #print(child_theta,rpm)
                break
                    
                    
    if (flag!=1):
        parent_list.append(parent_node)
        child_list.append(child_node)
        theta_list.append(child_theta)
        cost.append(round((child_cost+cost[x]),4))
        heuristic_list.append(round((child_cost+cost[x]+heu(child_node)),4))
        rpm_list.append(rpm)
    
    # print(child_node,rpm)  
    # print("child_list = ",child_list)
    
    return child_list,parent_list,cost,heuristic_list,theta_list,rpm_list


print("Enter the starting node coordinates")
xi=float(input("x =  "))
yi=float(input("y =  "))
ndi=[xi,yi]

print("Enter the goal node coordinates")
xg=float(input("x =  "))
yg=float(input("y =  "))
goal = [xg,yg]


if (workspace(goal[0],goal[1])==1):
    sys.exit("The given Goal node lies in the obstacle space 'or' outside the workspace")

    
if (workspace(ndi[0],ndi[1])==1):
    sys.exit("The given Initial node lies in the obstacle space 'or' outside the workspace")
    
"""
xg = 1
yg = 3
goal = [xg,yg]

ndi = [-4,1]
"""

ndi_theta = float(input("Enter the initial angle of turtlebot with x-axis in radians =  "))
rpm = [0,0]

# initializing the lists

parent_list = [ndi]
child_list = [ndi]
theta_list = [ndi_theta]
heuristic_list = [round(heu(ndi),4)]
rpm_list = [rpm]
visited_pn = []
visited_cn = []
visited_th = []
visited_cost = []
visited_heu = []
visited_rpm=[]

# setting a threshold for the goal node 

def goal_threshold(x,y):
    c=0
    if (x-goal[0])**2+(y-goal[1])**2<(0.1)**2:
        c=1
    return c

# setting a threshold for storing similar nodes

def threshold(x,y,visited_cn):
    c=0
    for i in range(0,len(visited_cn)):
        if (x-visited_cn[i][0])**2+(y-visited_cn[i][1])**2<(0.1)**2:
            c=1
    return c


x=0
cost=[0]

parent_node=ndi[:]
parent_theta = ndi_theta

check=0


while(check!=1):
    
    #  To check for nodes that should not be in workspace and check for the node with least heuristic cost: 
    
    # Child nodes for RPM as [0,5]

    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[0,5],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
            
     # Child nodes for RPM as [0,10]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[0,10],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
            
    # Child nodes for RPM as [5,10]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[5,10],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
    # Child nodes for RPM as [10,10]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[10,10],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
    # Child nodes for RPM as [5,5]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[5,5],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
    # Child nodes for RPM as [10,5]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[10,5],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
            
    # Child nodes for RPM as [10,0]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[10,0],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
    # Child nodes for RPM as [5,0]
    
    child_node_x, child_node_y, child_theta,rpm = diff_constraint(parent_node,[5,0],parent_theta)
    child_node = [child_node_x,child_node_y]
    child_cost = cost_to_come(child_node,parent_node)
    
    if (workspace(child_node_x, child_node_y)!=1):
        if threshold(child_node_x, child_node_y,visited_cn)==0:
            child_list,parent_list,cost, heuristic_list,theta_list,rpm_list=append(child_list,parent_list,cost,heuristic_list,x,parent_node,child_node,child_cost,child_theta,theta_list,rpm,rpm_list)
            
    visited_pn.append(parent_list.pop(x))
    visited_th.append(theta_list.pop(x))
    visited_cn.append(child_list.pop(x))
    visited_cost.append(cost.pop(x))
    visited_heu.append(heuristic_list.pop(x))
    visited_rpm.append(rpm_list.pop(x))
    
    
    if(goal_threshold(visited_cn[-1][0],visited_cn[-1][1])==1):
        print("Goal found")
        check=1
        
    #print("visited_cn[-1] = ",visited_cn[-1])
    #print("visited_rpm[-1] = ",visited_rpm[-1])

    
    if(check!=1):

        x=heuristic_list.index(min(heuristic_list))
        parent_node=child_list[x][:]
        parent_theta=float(theta_list[x])
        rpm=rpm_list[x][:]
        
    #print(rpm_list)
    #print("x = ",x)


# Node path and backtracking

seq=[]
rpm_seq=[]
seq.append(visited_cn[-1])
seq.append(visited_pn[-1])
rpm_seq.append(visited_rpm[-1])
a=visited_pn[-1]
b=visited_rpm[-1]
i=1
while(a!=ndi):
    if(visited_cn[-i]==a):
        seq.append(visited_pn[-i])
        rpm_seq.append(visited_rpm[-i])
        a=visited_pn[-i]
        b=visited_rpm[-i]
    i=i+1

#print(len(visited_cn))
print("\n Sequence = ",seq[::-1])
print("\n RPM = ",rpm_seq[::-1])
#print("\n rpm_list",visited_rpm)


# scatter plot for the nodes

for i in visited_cn:
    plt.scatter(i[0],i[1],color='g')
for i in seq:
    plt.scatter(i[0],i[1],color='r')
plt.show


# SAMPLE CODE:

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    #res,objs=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking)
    #if res==vrep.simx_return_ok:
    #    print ('Number of objects in the scene: ',len(objs))
    #else:
    #    print ('Remote API function call returned with error code: ',res)
    time = 0
#retrieve motor  handles
    errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking)
    errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking)
    r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_streaming)
    
    path_speeds = rpm_seq[::-1]


    for k in path_speeds:
        time = 0
        err_code1 = 1
        err_code2 = 2
        #print(type(k[0]))
        while(err_code1 != 0 and err_code2 != 0):
            err_code1 = vrep.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], vrep.simx_opmode_streaming)
            #print(err_code1)

            err_code2 = vrep.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], vrep.simx_opmode_streaming)
            #print(err_code2)

        r, signalValue = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

        while(time<2.0):

            r, signalValue2 = vrep.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', vrep.simx_opmode_buffer)

            time = signalValue2 - signalValue

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)

    
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

