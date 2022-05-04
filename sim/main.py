"""
File to run full path planning and balancing control sequence on hardware
"""
import sys
sys.path.append('../')
from simulator import *
from controllers import *
from dynamics import *
from path_planner import *
from Map import *
from obstacles import *
import time

#Define balance controller
balance_controller = CBF_QP(gains=[40, 20]) #use CBF QP controller with default parameters
path_planner_controller = PathPlannerController() #For now, just try the balancing dynamics, so use the zero planning controller
bicycle_controller = VehicleController(balance_controller, path_planner_controller) #create the full state controller

#define dynamics
dynamics = Bicycle() #define a bicycle object
#list of [radius,x,y]s for circular obstacles.
Circular_Obstacles_coordinates = [[1,4,4],[1,6,6]]
Circular_Obstacles_list = [Circular_Obstacle(x[0],x[1],x[2]) for x in Circular_Obstacles_coordinates]
my_blank_map = Map(0,10,0,10,Circular_Obstacles_list) #define a 10x10 map with no obstacles.

#Find optimal path from a start and goal position
q_start = np.array([0, 0, 1, 1, 0, 0, 0, 0])
q_goal = np.array([0, 0, 8, 8, 0, 0, 0, 0]) 
path_planner = PathPlanner(dynamics, q_start, q_goal,my_blank_map)
plan, inputs = path_planner.plan_to_pose()

#Create a bicycle object
bicycle = Hardware()

#Now, using this plan and these open loop inputs, begin the control loop where we do balancing
dt = path_planner.dt
for u_planner in inputs:
    #Move through the entire path planner path using the same time step as the planner
    t1 = time.perf_counter()
    #First, read the current state from the bicycle
    q_t = bicycle.read_state()
    #solve the CBF to get the balancing for this time step, then send the full input vector to the bicycle
    u_cbf = balance_controller.control_input(q_t, u_planner)
    #construct the input vector
    u_t = np.vstack((u_planner, u_cbf))
    #Send the input vector to the bicycle
    bicycle.send_input()

    #Wait the remainder of the time step to ensure synchronization
    t2 = time.perf_counter()
    if t2-t1<dt:
        time.sleep(dt-(t2-t1)) #sleep the remainder of the dt interval

#after planning finishes, indefinitely balance the bicycle
while True:
    q_t = bicycle.read_state()
    u_cbf = balance_controller.control_input(q_t, u_planner)
    #construct the input vector
    u_t = np.vstack((np.zeros(4, 1), u_cbf)) #put all zeros in planning inputs (keep the bicycle still)
    #Send the input vector to the bicycle
    bicycle.send_input()
    #Wait the remainder of the time step to ensure synchronization
    t2 = time.perf_counter()
    if t2-t1<dt:
        time.sleep(dt-(t2-t1)) #sleep the remainder of the dt inter
