from simulator import *
from controllers import *
from dynamics import *
from path_planner import *
"""
File to create and run simulations
"""
print("HELLO")
#first, define the different controllers for the vehicle
balance_controller = BalancePID(np.array([1, 1, 1]))
path_planner_controller = PathPlannerController() #For now, just try the balancing dynamics, so use the zero planning controller
bicycle_controller = VehicleController(balance_controller, path_planner_controller) #create the full state controller

#define dynamics
dynamics = Bicycle() #define a bicycle object

#Find optimal path from a start and goal position
q_start = np.array([0, 0, 0, 0, 0, 0, 0, 0])
q_goal = np.array([0, 0, 6, 5, 0, 0, 0, 0]) 
path_planner = PathPlanner(dynamics, q_start, q_goal)
plan, inputs = path_planner.plan_to_pose()

#path_planner.plot(plan, inputs)

#Next, define simulation object
sim = Simulation(bicycle_controller, dynamics, np.array([[np.pi/4], [np.pi/4], [0.5], [1], [np.pi/16], [0]]))
print("sim", sim)

#run the simulation
sim.simulate()
print("SIMULATION COMPLETED")

#animate path_planner plan
sim.animate_plan(plan,inputs)
#sim.animate_plan_3D(plan,inputs)

#plot the results

#sim.plot_results()
#sim.animate()
