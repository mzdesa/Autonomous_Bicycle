import numpy as np
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
    def x_tp1(self, x_t, u_t, t, dt):
        """
        Function to integrate dynamics forward in time for simulation.
        Returns x_t+1 given x_t
        Inputs:
        x_t: current state vector - torch tensor, (Nx1)
        u_t: current input vector - torch tensor, (Mx1)
        t: current time, float
        """
        return x_t + self.f(x_t, u_t, t)*dt
    def gen_dynamics(self, x_0, u_t):
        """
        Returns the full set of dynamics for the function across time period self.T
        Inputs:
        x_0: starting state
        u_t: function specifying inputs as a function of time
        """
        x_t = x_0
        x_next = x_0
        for t in self.times:
            x_t =  self.x_tp1(x_t, u_t(t), t)
            x_next = torch.cat(x_next, x_t, axis = 1) #concatenate with x_next array
        return x_next

class Bicycle(Dynamics):
    def __init__(self, state_bounds, input_bounds):
        """
        Initialize a dynamics object with bicycle dynamics
        """
    def gen_dynamics(l = 0.5, a = 0.5, b = 1, m = 10, m_f = 1, I_b = 5, I_f = 1):
        """
        Higher order function to generate the dynamics for the system.
        Calling gen_dynamics populates the f parameter in the bicycle dynamics and returns the function f

        Inputs:
        l: distance along the bicycle to the center of mass (m)
        a: distance to center of mass from back wheel in xy plane (m)
        b: distance between the two wheels of the bicycle (m)
        m: mass of the entire bicycle-flywheel system (kg)
        m_f: mass of the flywheel (kg)
        I_b: inertia bicycle-flywheel system
        I_f: moment of inertia of the f
        """
        def f(x, u, t):
            """
            Bicycle dynamics function in form x_dot = f(x, u)
            Bicycle state vector: x = [phi, x, y, psi, theta, theta_dot].T
            Bicycle input vector: u = [v, psi_dot, alpha_ddot].T

            State Descriptions:
            Phi: angle of frame relative to world in XY plane (rad)
            x: x coordinate of center of back wheel in world frame (m)
            y: y coordinate of center of back wheel in world frame (m)
            Psi: Steering angle relative to frame (rad)
            theta: angle of bicycle with respect to the vertical z axis (rad)

            Input Descriptions:
            v: velocity of the real wheel (m/s)
            psi_dot: steering rate (rad/s)
            alpha_ddot: acceleration of flywheel (rad/s^2)


            Function inputs:
            x: current state, (6x1) numpy vector
            u: current input, (3x1) numpy vector
            t: current time

            Function returns:
            x_dot = d/dt([phi, x, y, psi, theta, theta_dot].T) (6x1) numpy vector
            """
            g = 9.81 #grav. constant
            #create variables for each vector element
            #state elements
            phi = x[0, 1]
            x = x[1, 1]
            y = x[2, 1]
            psi = x[3, 1]
            theta = x[4, 1]
            theta_dot = x[5, 1]
            #input elements
            v = u[0, 1]
            psi_dot = u[1, 1]
            alpha_ddot = u[2, 1]

            #first, solve for the translation planar dynamics (follows a bicycle model)
            u_planar = u[0:2, 1] #take out the v and psi_dot
            #solve for the first 4 elements of x_dot
            x_04_dot = np.matmul(np.array([[np.tan(psi)/l, 0], [np.cos(phi), 0], [np.sin(phi), 0], [0, 1]]), u_planar)

            #now, solve for the balancing dynamics
            def gamma_r_solver():
                e_1 = np.array([[np.cos(phi), np.sin(phi), 0]]).T
                e_2 = np.array([[-np.sin(phi), np.cos(phi), 0]]).T
                E_1 = np.array([[1, 0, 0]]).T
                E_2 = np.array([[0, 1, 0]]).T
            gamma, r = gamma_r_solver()
            dV_ydt = 0
            theta_ddot = 1/I_b*(m*l*np.cos(theta)*(v**2/r*np.cos(gamma) + dV_ydt)+m*g*l*np.sin(theta) + I_f*alpha_ddot)
