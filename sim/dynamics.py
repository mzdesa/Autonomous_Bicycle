from cmath import inf
import numpy as np
from math import sin, cos, tan
import sympy
"""
Classes for generic dynamics models and sub-class for bicycle dynamics
Includes utilities useful for simulation.
"""

class Dynamics:
    """
    Dynamics class - defines dynamics model and associated parameters
    """
    def __init__(self, f, state_bounds, input_bounds):
        """
        Initializes a dynamics object
        f: dynamics function x_dot = f(x, u)
        """        
        self.f = f #x_dot = f(x, u, t) python function that returns numpy vector x_dot
        self.state_bounds = state_bounds
        self.input_bounds = input_bounds
        self.f_symbolic = None #Sympy representation of dynamics
    def q_tp1(self, q_t, u_t, t, dt):
        """
        Function to integrate dynamics forward in time for simulation.
        Returns q_t+1 given q_t
        Inputs:
        q_t: current state vector - numpy vector, (Nx1)
        u_t: current input vector - numpy vector, (Mx1)
        t: current time, float
        """
        #otherwise return Euler discretized next step
        return q_t + self.f(q_t, u_t)*dt
        
class Bicycle(Dynamics):
    def __init__(self, state_bounds = None, input_bounds = None, a_bar = 0.4, b = 1, c = 0.5, m = 10, I_bf = 5, I_f = 1):

        """
        Init function for bicycle dynamics
        Calling gen_dynamics populates the f parameter in the bicycle dynamics and returns the function f

        Inputs:
        state_bounds: (num_states x 2) numpy vector, row format: [low, high]
        input_bounds: (num_inputs x 2) numpy vector
        a_bar: vertical component of the center of mass coordinate of the entire bicycle-flywheel system
        b: horizontal distance between wheel centers
        c: horizonal distance from rear wheel to center of mass
        m: mass of the entire bicycle-flywheel system (kg)
        I_bf: inertia bicycle-flywheel system
        I_f: moment of inertia of the flywheel
        """
        
        #Define bicycle dynamics function
        def f(q, u):
            """
            Bicycle dynamics function in form q_dot = f(q, u)
            Bicycle state vector: q = [theta, theta_dot, x, y, psi, psi_dot].T
            Bicycle input vector: u = [v, v_dot, sigma, sigma_dot, alpha_ddot].T

            State Descriptions:
            theta: angle of frame relative to world in XY plane (rad)
            theta_dot: rate of change of theta
            x: x coordinate of center of back wheel in world frame (m)
            y: y coordinate of center of back wheel in world frame (m)
            psi: balancing angle of bicycle relative to vertical (rad)
            psi_dot: rate of change of psi (rad)

            Input Descriptions:
            v: velocity of the real wheel (m/s)
            v_dot: acceleration of the rear wheel (m/s^2)
            sigma: steering angle relative to frame (rad)
            sigma_dot: steering rate (rad/s)
            alpha_ddot: acceleration of flywheel (rad/s^2)

            Function inputs:
            q: current state, (6x1) numpy vector
            u: current input, (5x1) numpy vector
            t: current time

            Function returns:
            q_dot = d/dt([theta, theta_dot, x, y, psi, psi_dot].T) (6x1) numpy vector
            """
            g = 9.81 #grav. constant
            #create variables for each vector element
            #Inputs:
            v = u[0, 0]
            v_dot = u[1, 0]
            sigma = u[2, 0]
            sigma_dot = u[3, 0]
            alpha_ddot = u[4, 0]

            #assign variables to state vector elements
            theta = q[0, 0]
            theta_dot = q[1, 0]
            x = q[2, 0]
            y = q[3, 0]
            psi = q[4, 0]
            psi_dot = q[5, 0]
            #Calculate equivalent simple pendulum length a:
            a = (I_bf/m)

            #Assembly remaining derivative terms
            theta_ddot = v_dot*tan(sigma)/b + (v/b)*(sigma_dot)*(1/cos(sigma))**2
            x_dot = v*cos(theta)
            y_dot = v*sin(theta)
            psi_ddot = (1/(m*a**2))*(m*a**2*theta_dot**2*sin(psi)*cos(psi) + m*v*a*theta_dot*cos(psi)+m*c*a*theta_ddot*cos(psi)+I_f*alpha_ddot+m*a_bar*g*sin(psi))
            q_dot = np.array([[theta_dot, theta_ddot, x_dot, y_dot, psi_dot, psi_ddot]]).T
            return q_dot
        
        #define class parameters
        self.f = f
        self.state_bounds = state_bounds
        self.input_bounds = input_bounds
        
        #define symbolic variables for CBF CLF controller
        self.f_sym = None #generate with the function below
        self.g_sym = None


        #revisit this
        #For path planner
        self.b = b
        self.m = m
        self.I_bf = I_bf
        self.a = (I_bf/m)
        self.c = c
        self.a_bar = a_bar
        self.I_f = I_f
    
    def gen_sym(self):
        """
        Generates symbolic variables for bicycle dynamics. Populates f_sym and g_sym.
        Note: only run if using CBF-CLF controller, as computation is somewhat time-consuming.
        """

        