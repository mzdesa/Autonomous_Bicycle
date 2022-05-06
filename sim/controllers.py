from re import A
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

    def control_input(self, q, q_goal, inputs_from_path_planner):
        """
        Function to find the planning variable inputs to get q to q_goal
        Inputs:
        q: current state vector
        q_goal: goal state vector
        Returns: 
        vehicle input vector u: u = [v, v_dot, sigma, sigma_dot, alpha_ddot].T (numpy vector)
        """
        #controller_14 = self.path_planner_controller.control_input(q, q_goal) #get the first four variables in the input vector
        controller_14 = inputs_from_path_planner
        print("controller 14 shape", controller_14.shape)
        print("controller 14", controller_14)
        print("alpha ddot calculation beginning")
        alpha_ddot = self.balance_controller.control_input(q, controller_14) #get the acceleration of the flywheel for balancing - pass in the optimization based control inputs
        return np.vstack((controller_14, alpha_ddot)) #return the full input vector to the vehicle

# BALANCING CONTROLLERS

class BalanceController:
    """
    Generic skeleton class for a balancing controller - all controllers should follow this structure
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

class BalanceFullState:
    """
    Full state feedback controller for balancing dynamics
    Assumes negligible turning radius for linearization
    Assumes linearized pendulum dynamics
    """
    def __init__(self, dyn):
        """
        Init function for a full state balancing feedback controller
        Inputs:
        dyn: dynamics object (needed for linearization calculation)
        """
        self.dyn = dyn
        self.A = np.eye(6)
        self.B = np.zeros((6, 1))
        self.gains = np.zeros((6, 1))
    def linearize(self):
        """
        Compute the jacobian linearization of the system dynamics
        Inputs: None (refers to dyn object stored in class)
        Returns: A and B matrices for system
        """
        #first, declare symbolic variables for all of the dynamics
        pass

class CBF_QP:
    """
    Min Norm Control Barrier Function/QP controller for balancing
    """
    def __init__(self, dyn_param = [0.5, 0.4, 1, 10, 1, 0.5], gains = [40, 20], gamma=0.5, alpha=1, p=0.1, u_bounds = None):
        """
        Init function for a CLF CBF QP controller
        Inputs:
        dyn_param: [a, a_bar, b, m, I_f, c] list of dynamics parameters
        gains: list of Kp and Kd gains for K(q) ideal controller
        clf: control Lyapunov function (sympy)
        cbf: control Barrier function (sympy)
        gamma, alpha, p: optimization parameters
        u_bounds: bounds on the inputs to the vehicle
        """
        print("CBF QP INITIALIZATION")
        #unpack dynamics parameters
        a, a_bar, b, m, I_f, c = dyn_param
        grav = 9.81
        #Store Gains
        self.Kp = gains[0]
        self.Kd = gains[1]

        self.state_dimn = 6 #requires the full state vector to calculate balancing dynamics
        self.input_dimn = 1 #assume only a single input for now
        if u_bounds:
            self.u_min = u_bounds[0]
            self.u_max = u_bounds[1]
        else:
            #If u_bounds = None, set input bounds using inf
            self.u_min = np.ones((self.input_dimn, 1))*np.inf*-1
            self.u_max = np.ones((self.input_dimn, 1))*np.inf

        #Define symbolic variables
        self.q = sp.MatrixSymbol("q", 6, 1) #Note: use q for states, NOT x
        self.u = sp.MatrixSymbol("u", 5, 1) #Multi input system! Ensure the lie derivatives work with this

        #Define two CBFs (can also do HJ reachability analysis - look into this!)
        self.cbf1 = self.q[4, 0]+sp.pi/12 #enforce both barrier conditions simultaneously
        self.cbf2 = self.q[4, 0]-sp.pi/12

        #First order symbolic lie derivatives
        self.lfh1 = self.q[5, 0] #from symbolic calculation
        self.lfh2 = self.q[5, 0]
        self.lgh1 = sp.zeros(5, 1)
        self.lgh2 = sp.zeros(5, 1)

        #Higher order symbolic lie derivatives
        self.lf2h1 = -(m*a**2*sp.sin(self.q[4, 0])*self.q[1, 0]**2 + m*a*c*(sp.tan(self.u[2, 0]*self.u[1, 0]/b)+self.u[0, 0]*self.u[3, 0]/(b*sp.cos(self.u[2, 0])**2))*sp.cos(self.q[4, 0])+a_bar*grav*m*sp.sin(self.q[4, 0]))/(m*a**2)
        self.lf2h2 = (m*a**2*sp.sin(self.q[4, 0])*self.q[1, 0]**2 + m*a*c*(sp.tan(self.u[2, 0]*self.u[1, 0]/b)+self.u[0, 0]*self.u[3, 0]/(b*sp.cos(self.u[2, 0])**2))*sp.cos(self.q[4, 0])+a_bar*grav*m*sp.sin(self.q[4, 0]))/(m*a**2)
        self.lglfh1 = sp.Matrix([[sp.cos(self.q[4, 0])*self.q[1, 0]/a, 0, 0, 0, I_f/(m*a**2)]])
        self.lglfh2 = sp.Matrix([[sp.cos(self.q[4, 0])*self.q[1, 0]/a, 0, 0, 0, I_f/(m*a**2)]])

        #Lambda version of lie derivatives to be used with Casadi
        self.lf2h1_lambda = lambda q, u: -(m*a**2*np.sin(q[4, 0])*q[1, 0]**2 + m*a*c*(np.tan(u[2, 0]*u[1, 0]/b)+u[0, 0]*u[3, 0]/(b*np.cos(u[2, 0])**2))*np.cos(q[4, 0])+a_bar*grav*m*np.sin(q[4, 0]))/(m*a**2)
        self.lf2h2_lambda = lambda q, u: (m*a**2*np.sin(q[4, 0])*q[1, 0]**2 + m*a*c*(np.tan(u[2, 0]*u[1, 0]/b)+u[0, 0]*u[3, 0]/(b*np.cos(u[2, 0])**2))*np.cos(q[4, 0])+a_bar*grav*m*np.sin(q[4, 0]))/(m*a**2)
        self.lglfh1_lambda = lambda q: np.array([[np.cos(q[4, 0])*q[1, 0]/a, 0, 0, 0, I_f/(m*a**2)]])
        self.lglfh2_lambda = self.lglfh1_lambda
        self.cbf1_lambda = lambda q: q[4, 0]+np.pi/12
        self.cbf2_lambda = lambda q: -q[4, 0]+np.pi/12

        #Opti parameters
        self.gamma = gamma
        self.alpha = alpha
        self.p = p

    def get_lie_derivatives(self):
        """
        Function to generate necessary symbolic lie derivatives
        """
        def lie_derivative(f, V, q_dimn):
            """
            Function to take the lie derivative LfV
            Note: f, V functions must be functions of q = sympy.symbols('q')
            f: vector field function of x, sympy object
            V: V(q) function, sympy object
            q_dimn: dimension of the state vector of f
            Returns:
            LfV = (\partial(V)/\partial(q))f(q)
            """
            print("V", V)
            q = sp.MatrixSymbol("q", q_dimn, 1)
            print("X shape", q.shape)
            dVdq = V.jacobian(q)  # calculate the derivative of V wrt a vector x
            return dVdq * f

        #Define symbolic variables
        q = sp.MatrixSymbol('q', 6, 1)
        u = sp.MatrixSymbol('u', 5, 1)
        b, a, a_bar, m, grav, I_f, c = sp.symbols('b a a_bar m grav I_f c')

        #Define f(x) and g(x)
        theta_dot = q[1, 0]
        theta_ddot = u[1, 0]*sp.tan(u[2, 0])/b + (u[0,0]/b)*u[3, 0]/(sp.cos(u[2, 0]))**2
        # x_dot = u[0, 0]*sp.cos(q[0, 0])
        # y_dot = u[0, 0]*sp.sin(q[0, 0])
        # psi_dot = q[5, 0]
        # psi_ddot = (1/(m*a**2))*(m*a**2*theta_dot**2*sp.sin(q[4, 0])*sp.cos(q[4, 0]) + m*u[0, 0]*a*theta_dot*sp.cos(q[4, 0])+m*c*a*theta_ddot*sp.cos(q[4, 0])+I_f*u[4, 0]+m*a_bar*grav*sp.sin(q[4, 0]))
        f = sp.Matrix([[q[1, 0], 0, 0, 0, q[5, 0], (1/(m*a**2))*(m*a**2*theta_dot**2*sp.sin(q[4, 0])*sp.cos(q[4, 0]) +m*c*a*theta_ddot*sp.cos(q[4, 0])+m*a_bar*grav*sp.sin(q[4, 0]))]]).T
        g = sp.Matrix([[0, 0, 0, 0, 0], [0, 0, u[1, 0]/b, (1/b)*u[0, 0]/(sp.cos(u[2, 0]))**2, 0],[sp.cos(q[0, 0]), 0, 0, 0, 0], [sp.sin(q[0, 0]), 0, 0, 0, 0], [0, 0, 0, 0, 0], [(1/(m*a**2))*m*a*theta_dot*sp.cos(q[4, 0]), 0, 0, 0, I_f/(m*a**2)]])

        #Define Barrier functions
        h1 = sp.Matrix([[-q[4, 0]+sp.pi/12]]) #enforce both barrier conditions simultaneously
        h2 = sp.Matrix([[q[4, 0]-sp.pi/12]])

        #Take Lie Derivatives
        lfh1 = lie_derivative(f, h1, 6)
        lfh2 = lie_derivative(f, h2, 6)
        #calculate lie derivatives for g
        # lgh1 = lie_derivative(g, h1, 6)
        # lgh2 = lie_derivative(g, h2, 6)

        #Keep going to find the relative degree
        _ = lie_derivative(g, lfh1, 6) #the input appears in this step - thus, the relative degree is 2

        #The system THEREFORE has a relative degree of 2!
        #Calculate all 2nd lie derivatives necessary
        lf2h1 = lie_derivative(f, lfh1, 6)
        lf2h2 = lie_derivative(f, lfh2, 6)
        lglfh1 = lie_derivative(g, lfh1, 6)
        lglfh2 = lie_derivative(g, lfh2, 6)
    
    def control_input(self, q_t, opti_inputs):
        """
        Function to get control input given current state according to CLF CBF QP
        Uses Casadi for optimization
        Inputs:
        q_t: state vector at current time step, (state_dimn x 1) numpy vector
        opti_inputs: input vector returned from optimization!
        """
        #set up variables for easier access to class parameters
        q_t = q_t.reshape((q_t.shape[0], 1)) #ensure x_t has correct dimensions
        print("qt", q_t)
        h1 = self.cbf1_lambda(q_t) #cbf function value
        h2 = self.cbf2_lambda(q_t)

        #Check for safety condition (h(x)>=0)
        if h1<0 or h2<0:
            print(h1)
            print(h2)
            print("Current state vector", q_t)
            print("unsafe")
            # return (None, "unsafe")
            return 0
        
        #set up Casadi optimization
        opti = ca.Opti()

        #set up optimization variables
        u = opti.variable(5, 1) #input variable - note: first time trying MIMO system
        alpha = self.alpha #tunable constant

        #Calculate Lie derivatives
        lf2h1 = self.lf2h1_lambda(q_t, u)
        lf2h2 = self.lf2h2_lambda(q_t, u)
        lglfh1 = self.lglfh1_lambda(q_t)
        lglfh2 = self.lglfh2_lambda(q_t)

        #Enforce optimization input constraints - enforce for all but alpha_ddot
        opti.subject_to(u[0, 0] == opti_inputs[0, 0])
        opti.subject_to(u[1, 0] == opti_inputs[1, 0])
        opti.subject_to(u[2, 0] == opti_inputs[2, 0])
        opti.subject_to(u[3, 0] == opti_inputs[3, 0])

        #Enforce Higher order CBF constraints
        opti.subject_to(lf2h1+ca.mtimes(lglfh1, u)>= -alpha*h1) #cbf constraint 1
        opti.subject_to(lf2h2+ca.mtimes(lglfh2, u)>= -alpha*h2) #cbf constraint 2

        #Input bound constraints:
        if self.u_max < np.inf:
            #for finite bounds, enforce these constraints
            opti.subject_to(-self.u_min <= u)
            opti.subject_to(self.u_max >= u)

        #Set up cost function
        #first, calculate the value of k(q) from a PD controller
        #ideal input without safety constraints
        k_q = self.Kp*(0-q_t[4, 0]) + self.Kd*(0-q_t[5, 0]) #gives PD control in psi
        cost = ca.mtimes((u-k_q).T, (u-k_q)) #Min norm cost function

        #set up optimization problem
        opti.minimize(cost)
        p_opts = {"expand": False}
        s_opts = {"max_iter": 1e4, "acceptable_tol": 1e10}
        opti.solver("ipopt", p_opts, s_opts)

        #solve optimization
        sol = opti.solve()
        u_opt = sol.value(u)

        #return the optimal input!
        return u_opt[-1] #the last element should be the optimal alpha_ddot

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
    def control_input_from_path_planner(inputs):
        return np.zeros((4, 1)) # do nothing

if __name__ == '__main__':
    """
    Run a test of the CBF controller
    """
    a_bar = 0.4
    b = 1
    c = 0.5
    m = 10
    I_bf = 5
    I_f = 1
    a = (I_bf/m)
    controller = CBF_QP([a, a_bar, b, m, I_f, c]) #input dynamics parameters, leave the rest default
    alpha_ddot_opt = controller.control_input(np.ones((6, 1))*0.1, np.ones((4, 1))) #leave a very small initial condition and an opti input
    print(alpha_ddot_opt)