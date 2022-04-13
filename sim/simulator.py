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

    def plot_results(self, plot_args = [True, True, True]):
        """
        Plot sequence:
        1) Plot evolution of each state variable in time
        2) Then, plot evolution of inputs in time
        3) Then, plot x and y against each other to show path in space
        Notes:
        Function assumes that the self.states, self.inputs, and self.times variables have already been populated
        Recall: q = [theta, theta_dot, x, y, psi, psi_dot].T
                u = [v, v_dot, sigma, sigma_dot, alpha_ddot].T
        Inputs:
        plot_args: 3x1 Boolean python list - tells function which of the three plots to show.
        """
        if plot_args[0]:
            #first, plot the state variables in a 6x1 subplot
            fig, axs = plt.subplots(6)
            fig.suptitle('Evolution of Bicycle States in Time')
            xlabels = 'Time (s)'
            ylabels = ['theta', 'theta_dot', 'x', 'y', 'psi', 'psi_dot']
            for i in range(6):
                axs[i].plot(self.states[i, :, self.times])
                axs[i].set(xlabel=xlabels, ylabel=ylabels[i]) #pull labels from the list above
            plt.show()

        if plot_args[1]:
            #next, plot the input variables in a 5x1 subplot
            fig, axs = plt.subplots(5)
            fig.suptitle('Evolution of Bicycle Inputs in Time')
            ylabels = ['v', 'v_dot', 'sigma', 'sigma_dot', 'alpha_ddot']
            for i in range(5):
                axs[i].plot(self.states[i, :, self.times])
                axs[i].set(xlabel=xlabels, ylabel=ylabels[i])
            plt.show()

        if plot_args[2]:
            #next, plot the x and y against each other to show path in space
            plt.plot(self.states[2], self.states[3])
            plt.title("XY Path of bicycle rear wheel")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.show()

    def animate(self):
        """
        Runs two animations of the bicycle
        1) Lateral dynamics from above in XY frame
        2) Balancing dynamics in frame traveling with bicycle
        """
        pass


    def run(self, T = 10, plot_results = True, animate = True):
        """
        Master simulation running method. Calls other class methods.
        Inputs:
        T: total time period for simulation
        plot_results: Boolean variable - show graphs or don't
        animate: Boolean variable - show animation or don't

        Returns: None
        """
        self.simulate(T) #first run simulation
        self.plot_results() #next plot results

