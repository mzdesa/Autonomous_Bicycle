from cmath import inf
import numpy as np
from math import sin, cos, tan, atan
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
    def x_tp1(self, x_t, u_t, t, dt):
        """
        Function to integrate dynamics forward in time for simulation.
        Returns x_t+1 given x_t
        Inputs:
        x_t: current state vector - torch tensor, (Nx1)
        u_t: current input vector - torch tensor, (Mx1)
        t: current time, float
        """
        #otherwise return Euler discretized next step
        return x_t + self.f(x_t, u_t, t)*dt
        
class Bicycle(Dynamics):
    def __init__(self, state_bounds = None, input_bounds = None, l = 0.5, a = 0.5, b = 1, m = 10, m_f = 1, I_b = 5, I_f = 1):

        """
        Init function for bicycle dynamics
        Calling gen_dynamics populates the f parameter in the bicycle dynamics and returns the function f

        Inputs:
        state_bounds: (num_states x 2) numpy vector, row format: [low, high]
        input_bounds: (num_inputs x 2) numpy vector
        l: distance along the bicycle to the center of mass (m)
        a: distance to center of mass from back wheel in xy plane (m)
        b: distance between the two wheels of the bicycle (m)
        m: mass of the entire bicycle-flywheel system (kg)
        m_f: mass of the flywheel (kg)
        I_b: inertia bicycle-flywheel system
        I_f: moment of inertia of the f
        """
        #Define bicycle dynamics function
        def f(q, u, t):
            """
            Bicycle dynamics function in form q_dot = f(q, u)
            Bicycle state vector: q = [phi, phi_dot, x, y, psi, theta, theta_dot].T
            Bicycle input vector: u = [v, v_dot, psi_dot, alpha_ddot].T

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
            x: current state, (7x1) numpy vector
            u: current input, (4x1) numpy vector
            t: current time

            Function returns:
            x_dot = d/dt([phi, x, y, psi, theta, theta_dot].T) (6x1) numpy vector
            """
            g = 9.81 #grav. constant
            #create variables for each vector element
            #state elements
            phi = q[0, 0]
            phi_dot = q[1, 0]
            x = q[2, 0]
            y = q[3, 0]
            psi = q[4, 0]
            theta = q[5, 0]
            theta_dot = q[6, 0]
            #input elements
            v = u[0, 0]
            v_dot = u[1, 0]
            psi_dot = u[2, 0]
            alpha_ddot = u[3, 0]

            #first, get relevant angles
            if v == 0:
                v += 0.0000001 #buffer to avoid angle solver divergence
            gamma = atan(a*phi_dot/v)
            #Next, calculate radius of curvature
            try:
                r = a/sin(gamma) #radius of curvature for center of mass
            except ZeroDivisionError:
                r = inf #set radius of curvature to be infinity (straight line)
            #Now, solve for derivative of each variable in q
            # phi_dot = phi_dot (already have from q vector)
            sec = lambda x: 1/cos(x)
            phi_ddot = v_dot*tan(psi)/l + v*psi_dot*(sec(psi))**2/l
            x_dot = v*cos(phi)
            y_dot = v*sin(phi)
            # psi_dot = psi_dot (already have from u vector)
            # theta_dot - theta_dot (already have from q vector)
            #Calculate dVy/dt before finding theta_ddot
            dV_ydt = a*phi_ddot #Assume zero for now to keep state vector compact
            theta_ddot = 1/I_b*(m*l*cos(theta)*(v**2/r*cos(gamma) + dV_ydt)+m*g*l*sin(theta) + I_f*alpha_ddot)
            #Assembly full state vector
            x_dot = np.array([[phi_dot, phi_ddot, x_dot, y_dot, psi_dot, theta_dot, theta_ddot]]).T
            return x_dot
        
        #define class parameters
        self.f = f
        self.state_bounds = state_bounds
        self.input_bounds = input_bounds