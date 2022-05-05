from simulator import *
from controllers import *
from dynamics import *
from path_planner import *
from Map import *
from obstacles import *
"""
File to create and run simulations
"""
print("HELLO")
#first, define the different controllers for the vehicle
# balance_controller = BalancePID(np.array([1, 1, 1]))
balance_controller = CBF_QP() #use CBF QP controller with default parameters
path_planner_controller = PathPlannerController() #For now, just try the balancing dynamics, so use the zero planning controller
bicycle_controller = VehicleController(balance_controller, path_planner_controller) #create the full state controller
#IM NOT SURE WE ACTUALLY USE THE PATH PLANNER CONTROLLER CLASS

#define dynamics
dynamics = Bicycle() #define a bicycle object
#list of [radius,x,y]s for circular obstacles.
Circular_Obstacles_coordinates = [[0.5,4,4],[0.5,7,6]]
Circular_Obstacles_list = [Circular_Obstacle(x[0],x[1],x[2]) for x in Circular_Obstacles_coordinates]
my_blank_map = Map(0,11,0,11,Circular_Obstacles_list) #define a 10x10 map with no obstacles.

#Find optimal path from a start and goal position
q_start = np.array([0, 0, 1, 1, 0, 0, 0, 0])
q_goal = np.array([0, 0, 9, 9, 0, 0, 0, 0]) 
path_planner = PathPlanner(dynamics, q_start, q_goal,my_blank_map)
plan, inputs = path_planner.plan_to_pose()

#path_planner.plot(plan, inputs)
#THE BICYCLE CONTROLLER IS MEANT TO CONTAIN THE PATH PLANNER BUT THEY SEEM TOTALLY DISCONNECTED.
#Next, define simulation object
# sim = Simulation(bicycle_controller, dynamics, np.array([[np.pi/4], [np.pi/4], [0.5], [1], [np.pi/16], [0]]),my_blank_map)
sim = Simulation(bicycle_controller, dynamics,  np.array([[0, 0, 1, 1, 0, 0]]).T,my_blank_map)

print("sim", sim)

#run the simulation
sim.simulate()
print("SIMULATION COMPLETED")

#THE plan IS PASSED DIRECTLY FROM THE PATH PLANNER, THE
#PATH PLANNING ASPECT OF THE BICYCLE CONTROLLER CLASS IS 
#NOT USED AT ALL

#animate path_planner plan
sim.animate_plan_2D(plan,inputs)

sim.animate_plan_3D(plan,inputs)

#plot the results

sim.plot_results()
#sim.animate()
