#python dependencies
import numpy as np
import matplotlib.pyplot as plt
#our dependencies
from controllers import *
from dynamics import *

"""
Create a simulation for the bicycle path planning and dynamics.
"""

class Simulation:
    """
    Class to manage simulations.
    """
    def __init__(self, bicycle_controller, dynamics, q0, dt = 0.01):
        """
        Initialize a Simulation object
        Inputs:
        bicycle_controller: a controller of class VehicleController
        dynamics: bicycle dynamics object
        q0: initial state of the vehicle
        dt: time step (seconds)
        T: period of simulation (seconds)
        """
        self.controller = bicycle_controller 
        self.dynamics = dynamics
        self.q0 = q0
        self.dt = dt
        self.states = [q0] #(state_dimn x N ) vector of states over time
        self.inputs = [] #(control_dimn x N) vector of control inputs over time
        self.times = None

    def simulate(self, T = 10):
        """
        Run the simulation, populate the states and inputs variables
        Inputs:
        T: period of simulation, default = 10s
        Returns:
        states: (state_dimn x N ) vector of states over time
        inputs: (control_dimn x N) vector of control inputs over time
        """
        q_goal = np.zeros((6, 1)) #SUBJECT TO CHANGE BASED ON PATH PLANNER - here, just leave as a vector of zeroes.
        
        q_t = self.q0  #initialize x_t as q0
        times = np.arange(0, T, self.dt) #create an array of times to go through
        self.times = times #store in class variable

        for t in times:
            u_t = self.controller.control_input(q_t, q_goal) #get the current input
            self.inputs = u_t #begin populating the input vector
            #Simulate the dynamics by getting the next step
            q_t = self.dynamics.q_tp1(q_t, u_t, t) #get the next state by calling q_tp1 dynamics method

            #add the state and the input to the object parameters
            self.states.append(q_t)
            self.inputs.append(q_t)

        return self.states, self.inputs, times

    def plot_results(self):
        #Plot evolution of each state variable in time.
        pass


