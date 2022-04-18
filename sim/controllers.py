import casadi as ca
import numpy as np
import sympy as sp
from obstacles import *
from dynamics import *

"""
File to define ALLvehicle controllers, which produce the full input vector of the vehicle given current and desired states.
*************************
Classes:
*************************
VehicleController:
Full state controllers for the bicycle. They give appropriate cartesian and balance control commands.
By calling planner_input, you will receive the full input vector u_t.
*************************
BalanceController:
Class for a stabilization controller
Includes child classes for PID, CBF-CLF, etc.
Note: the controllers only aim to stabilize psi, and do not do path planning.
*************************
PathPlannerController:
Class for finding path planning inputs
"""
class VehicleController:
    """
    Generic skeleton class for full state control. Other VehicleController classes should follow this format.
    VehicleControllers take in a goal state and return the set of non-balancing inputs to get to the goal state.
    """
    def __init__(self, balance_controller, path_planner_controller):
        """
        Initialize a vehicle controller object.
        Inputs:
        balance_controller: a balancing controller object that returns the appropriate alpha_ddot to stabilize the bicycle
        path_planner_controller: controller that returns the appropriate XY planning inputs 
        """
        self.balance_controller = balance_controller
        self.path_planner_controller = path_planner_controller

    def control_input(self, q, q_goal):
        """
        Function to find the planning variable inputs to get q to q_goal
        Inputs:
        q: current state vector
        q_goal: goal state vector
        Returns: 
        vehicle input vector u: u = [v, v_dot, sigma, sigma_dot, alpha_ddot].T (numpy vector)
        """
        alpha_ddot = self.balance_controller.control_input(q, q_goal) #get the acceleration of the flywheel for balancing
        controller_14 = self.path_planner_controller.control_input(q, q_goal) #get the first four variables in the input vector
        return np.vstack((controller_14, alpha_ddot)) #return the full input vector to the vehicle

# BALANCING CONTROLLERS

class BalanceController:
    """
    Generic skeleton class for a balancing controller - all controllers must follow this structure
    """
    def __init__(self, gains = np.zeros((6, ))):
        """
        Inputs:
        gains: numpy matrix of gains
        """
        self.gains = gains
    def control_input(self, q, qd):
        """
        Return the control input for the controller
        q: current state vector of the bicycle
        qd: desired state vector of the bicycle

        Returns:
        alpha_ddot input to the system
        """
        return 0

class BalancePID:
    def __init__(self, gains = np.array([1, 1, 1])):
        """
        Initialize a PID controller object for the bicycle
        Inputs: 
        gains: [Kp, Kd, Ki] - (3, ) numpy vector
        """
        self.gains = gains

    def control_input(self, q, qd, psi_int_err = 0):
        """
        Return the PD control input of the system
        Inputs:
        q: current state vector of the bicycle
        qd: desired state vector of the bicycle
        psi_int_err: integral error of psi (by default is a zero term - gives PD control)
        Note: must supply an argument for integral error for PID. Otherwise, controller will function as PD.
        Returns:
        alpha_ddot input to the system
        """
        #define the proportional and derivative gains
        Kp = self.gains[0]
        Kd = self.gains[1]
        Ki = self.gains[2]

        #pull the necessary state variables from q, qd
        psi, psi_dot = q[4], q[5]
        psi_d, psi_d_dot = qd[4], qd[5]

        #define the proportional and derivative errors in psi
        psi_err = psi_d - psi
        psi_dot_err = psi_d_dot - psi_dot

        return Kp*psi_err + Kd*psi_dot_err + Ki*psi_int_err #PID controller output

class BalancePD:
    def __init__(self, gains = np.array([1, 1, 1])):
        """
        Initialize a PD controller object for the bicycle
        Inputs: 
        gains: [Kp, Kd, Ki] - (3, ) numpy vector
        """
        self.gains = gains

    def control_input(self, q, qd):
        """
        Return the PD control input of the system
        Inputs:
        q: current state vector of the bicycle
        qd: desired state vector of the bicycle

        Returns:
        alpha_ddot input to the system
        """
        return BalancePID.control_input(q, qd, 0)

#CARTESIAN PLANNING CONTROLLERS

"""
NOTE: Here's what I think a good structure for the PathPlanner controller might be:
it should take in the current and desired goal from an optimization-based path planner and THEN return the inputs to achieve small steps towards the goal
"""
class PathPlannerController:
    """
    Generic path planner controller class
    Returns a 4x1 numpy vector corresponding to all of the cartesian planning inputs
    """
    def __init__(self):
        pass
    def control_input(self, q, qd):
        return np.zeros((4, 1)) # do nothing