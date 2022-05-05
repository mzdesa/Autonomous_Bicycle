from simulator import *
from controllers import *
from dynamics import *
from path_planner import *
from Map import *
from obstacles import *
import socket
"""
File to create and run simulations
"""
print("HELLO")
#first, define the different controllers for the vehicle
balance_controller = BalancePID(np.array([1, 1, 1]))
path_planner_controller = PathPlannerController() #For now, just try the balancing dynamics, so use the zero planning controller
bicycle_controller = VehicleController(balance_controller, path_planner_controller) #create the full state controller
#IM NOT SURE WE ACTUALLY USE THE PATH PLANNER CONTROLLER CLASS


#define dynamics
dynamics = Bicycle() #define a bicycle object
#list of [radius,x,y]s for circular obstacles.
Circular_Obstacles_coordinates = [] #[[1,4,4],[1,6,6]]
Circular_Obstacles_list = [Circular_Obstacle(x[0],x[1],x[2]) for x in Circular_Obstacles_coordinates]
my_blank_map = Map(0,10,0,10,Circular_Obstacles_list) #define a 10x10 map with no obstacles.

#Find optimal path from a start and goal position
q_start = np.array([0, 0, 1, 1, 0, 0, 0, 0])
q_goal = np.array([0, 0, 8, 8, 0, 0, 0, 0]) 
path_planner = PathPlanner(dynamics, q_start, q_goal,my_blank_map)
plan, inputs = path_planner.plan_to_pose()

print("inputs:", inputs)
HOST = "raspberrypi.local"
PORT = 6002


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(bytes(str(np.array([1763.287, 3.2])), 'utf-8'))
    print("data sent")