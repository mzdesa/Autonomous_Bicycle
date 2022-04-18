from simulator import *
from controllers import *
from dynamics import *
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

#Next, define simulation object
sim = Simulation(bicycle_controller, dynamics, np.array([[np.pi/4], [np.pi/4], [0.5], [1], [np.pi/16], [0]]))
print("sim", sim)

#run the simulation
sim.simulate()

print("SIMULATION COMPLETED")
#plot the results

#sim.plot_results()
sim.animate()
