from casadi import Opti, vertcat, mtimes
import casadi as cas
import numpy as np
import matplotlib.pyplot as plt


from dynamics import *
from simulator import *


#Path planner from homework with small changes to fit our model

class PathPlanner:
    """
    Path Planner class - Calculates the optimal trajectory using the casadi NLP solver. 
    """
    def __init__(self, dynamics, q_start, q_goal, map,
                q_lb = [-100, -100, -5, -1, -100, -100, -100, -100], q_ub = [100, 100, 10, 10, 100, 100, 100, 100],
                u_lb = [-10, -10, -10], u_ub = [10, 10, 10], n=1000, dt=0.01):
        """
        Initializes the planner with constraints.
        Inputs:
        q_start: starting state
        q_goal: goal state
        q_lb, q_ub: the state lower and upper bounds repspectively.
        u_lb, u_ub: the input lower and upper bounds repspectively.
        obs_list: the list of obstacles, each given as a tuple (x, y, r).
        n: the number of timesteps.
        dt: the discretization timestep."""

        self.dynamics = dynamics
        self.q_start = q_start
        self.q_goal =q_goal
        self.q_lb = q_lb
        self.q_ub = q_ub
        self.u_lb = u_lb
        self.u_ub = u_ub
        self.map = map
        self.n = n
        self.dt= dt


    def path_planner_q_tp1(self, q, u):
        """
        Implements the discrete time dynamics of your robot.
        i.e. this function implements F in

        q_{t+1} = F(q_{t}, u_{t})
        Inputs:
        dt: discretization timestep.
        q: array of casadi MX.sym symbolic variables [theta, theta_dot, x, y]
        u: array of casadi MX.sym symbolic variables [v, v_dot, sigma, sigma_dot]
        """
        g = 9.81 #grav. constant
        b = self.dynamics.b
        m = self.dynamics.m
        a = self.dynamics.a
        c = self.dynamics.c
        a_bar = self.dynamics.a_bar
        I_f = self.dynamics.I_f

        #create variables for each vector element
        #Inputs:
        v_dot, sigma_dot, alpha_ddot= u[0], u[1], u[2]

        #assign variables to state vector elements
        theta, sigma, x, y, theta_dot, psi, psi_dot, v = q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7]


        #Assembly remaining derivative terms
        theta_dot = v*cas.tan(sigma)/self.dynamics.b
        x_dot = v*cas.cos(theta)
        y_dot = v*cas.sin(theta)
        theta_ddot = v_dot*cas.tan(sigma)/b + (v/b)*(sigma_dot)*(1/cas.cos(sigma))**2
        psi_ddot = (1/(m*a**2))*(m*a**2*theta_dot**2*cas.sin(psi)*cas.cos(psi) + m*v*a*theta_dot*cas.cos(psi)+m*c*a*theta_ddot*cas.cos(psi)+I_f*alpha_ddot+m*a_bar*g*cas.sin(psi))
        
        q_dot = vertcat(theta_dot, sigma_dot, x_dot, y_dot, theta_ddot, psi_dot, psi_ddot, v_dot)
        return q + self.dt*q_dot

    def initial_cond(self):
        """
        Construct an initial guess for a solution to "warm start" the optimization.

        An easy way to initialize our optimization is to say that our robot will move 
        in a straight line in configuration space. Of course, this isn't always possible 
        since our dynamics are nonholonomic, but we don't necessarily need our initial 
        condition to be feasible. We just want it to be closer to the final trajectory 
        than any of the other simple initialization strategies (random, all zero, etc).

        We'll set our initial guess for the inputs to zeros to hopefully bias our solver into
        picking low control inputs

        n is the number of timesteps.

        This function will return two arrays: 
            q0 is an array of shape (4, n+1) representing the initial guess for the state
            optimization variables.

            u0 is an array of shape (2, n) representing the initial guess for the state
            optimization variables.
        """
        n = self.n
        q0 = np.zeros((8, n + 1))
        u0 = np.zeros((3, n))

        thetas = np.linspace(self.q_start[0],self.q_goal[0],n+1).reshape((1,n+1))
        sigma = np.linspace(self.q_start[1],self.q_goal[1],n+1).reshape((1,n+1))
        xs = np.linspace(self.q_start[2],self.q_goal[2],n+1).reshape((1,n+1))
        ys = np.linspace(self.q_start[3],self.q_goal[3],n+1).reshape((1,n+1))
        theta_dots = np.linspace(self.q_start[4],self.q_goal[4],n+1).reshape((1,n+1))
        psis = np.linspace(self.q_start[5],self.q_goal[5],n+1).reshape((1,n+1))
        psi_dots = np.linspace(self.q_start[6],self.q_goal[6],n+1).reshape((1,n+1))
        vs = np.linspace(self.q_start[7],self.q_goal[7],n+1).reshape((1,n+1))


        q0 = vertcat(thetas, sigma, xs, ys, theta_dots, psis, psi_dots, vs)

        return q0, u0

    def objective_func(self, q, u, q_goal, Q, R, P):
        """
        Implements the objective function. q is an array of states and u is an array of inputs. Together,
        these two arrays contain all the optimization variables in our problem.

        In particular, 

        q has shape (4, N+1), so that each column of q is an array q[:, i] = [q0, q1, q2, q3]

        u has shape (4, N), so that each column of u is an array u[:, i] = [u1, u2, u3, u4]

        This function should create an expression of the form

        sum_{i = 1, ..., N} ((q(i) - q_goal)^T * Q * (q(i) - q_goal) + (u(i)^T * R * u(i)))
        + (q(N+1) - q_goal)^T * P * (q(N+1) - q_goal)

        Note: When dealing with casadi symbolic variables, you can use @ for matrix multiplication,
        and * for standard, numpy-style element-wise (or broadcasted) multiplication.
        """

        n = q.shape[1] - 1
        obj = 0
        print("goal", q_goal, q)
        for i in range(n):
            qi = q[:, i]
            ui = u[:, i]

            # Define one term of the summation here: ((q(i) - q_goal)^T * Q * (q(i) - q_goal) + (u(i)^T * R * u(i)))
            term = mtimes(mtimes((qi - q_goal).T,Q),(qi - q_goal)) + mtimes(mtimes(ui.T,R),ui)
            obj += term

        q_last = q[:, n]
        # Define the last term here: (q(N+1) - q_goal)^T * P * (q(N+1) - q_goal)
        term_last = mtimes(mtimes((q_last - q_goal).T,P),(q_last - q_goal))
        obj += term_last
        return obj

    def constraints(self, q, u):
        """
        Constructs a list where each entry is a casadi.MX symbolic expression representing
        a constraint of our optimization problem.

        q has shape (4, N+1), so that each column of q is an array q[:, i] = [q0, q1, q2, q3]
        (i.e. [x, y, theta, phi]), the state at time-step i. 

        u has shape (2, N), so that each column of u is an array u[:, i] = [u1, u2], the two control inputs 
        (velocity and steering) of the bicycle model.

        q_lb is a size (4,) array [x_lb, y_lb, theta_lb, phi_lb] containing lower bounds for each state variable.

        q_ub is a size (4,) array [x_ub, y_ub, theta_ub, phi_ub] containing upper bounds for each state variable.

        u_lb is a size (2,) array [u1_lb, u2_lb] containing lower bounds for each input.

        u_ub is a size (2,) array [u1_ub, u2_ub] containing upper bounds for each input.

        obs_list is a list of obstacles, where each obstacle is represented by  3-tuple (x, y, r)
                representing the (x, y) center of the obstacle and its radius r. All obstacles are modelled as
                circles.

        q_start is a size (4,) array representing the starting state of the plan.

        q_goal is a size (4,) array representing the goal state of the plan.

        L is the axel-to-axel length of the car.

        dt is the discretization timestep.

        """
        constraints = []

        # # State constraints
        #constraints.extend([self.q_lb[0] <= q[0, :], q[0, :] <= self.q_ub[0]])
        #constraints.extend([self.q_lb[1] <= q[1, :], q[1, :] <= self.q_ub[1]])
        #constraints.extend([self.q_lb[2] <= q[2, :], q[2, :] <= self.q_ub[2]])
        #constraints.extend([self.q_lb[3] <= q[0, :], q[3, :] <= self.q_ub[0]])
        #constraints.extend([self.q_lb[4] <= q[1, :], q[4, :] <= self.q_ub[1]])
        #constraints.extend([self.q_lb[5] <= q[2, :], q[5, :] <= self.q_ub[2]])
        #constraints.extend([self.q_lb[6] <= q[2, :], q[6, :] <= self.q_ub[2]]) 

        
        # # Input constraints
        #constraints.extend([self.u_lb[0] <= u[0, :], u[0, :] <= self.u_ub[0]])
        #constraints.extend([self.u_lb[1] <= u[1, :], u[1, :] <= self.u_ub[1]])
        #constraints.extend([self.u_lb[0] <= u[2, :], u[2, :] <= self.u_ub[2]])
        #constraints.extend([self.u_lb[1] <= u[3, :], u[3, :] <= self.u_ub[3]])

        # Dynamics constraints
        for t in range(q.shape[1] - 1):
            q_t   = q[:, t]
            q_tp1 = q[:, t + 1]
            u_t   = u[:, t]
            constraints.append(q_tp1 == self.path_planner_q_tp1(q_t, u_t)) # You should use the bicycle_robot_model function here somehow.



            #print("constraints.length before obstacles",len(constraints))
            #Obstacle constraints
            #NEED TO FIX THIS, ASSUMES OBSTACLES ARE ALL CIRCLES.
            for ob in self.map.obstacles:
                #print("self.map.obstacles.length",len(self.map.obstacles))
                if isinstance(ob,Circular_Obstacle):
                    x = ob.x
                    y = ob.y
                    r = ob.radius
                    print(x,y,r)
                    constraints.append((q[2,t]-x)**2 + (q[3,t]-y)**2 > r**2) # Define the obstacle constraints.
            #print("constraints.length after obstacles",len(constraints))

        #Initial and final state constraints
        
        constraints.append(self.q_start == q[:,0]) # Constraint on start state.
        constraints.append(self.q_goal == q[:,q.shape[1]-1]) # Constraint on final state.

        return constraints

    def plan_to_pose(self):
        """
        Plans a path from q_start to q_goal.

        q_lb, q_ub are the state lower and upper bounds repspectively.
        u_lb, u_ub are the input lower and upper bounds repspectively.
        obs_list is the list of obstacles, each given as a tuple (x, y, r).
        L is the length of the car.
        n is the number of timesteps.
        dt is the discretization timestep.

        Returns a plan (shape (4, n+1)) of waypoints and a sequence of inputs
        (shape (2, n)) of inputs at each timestep.
        """
        n = self.n
        opti = Opti()

        q = opti.variable(8, n + 1)
        #need to change the opti variables
        #u[0] u[3]
        u = opti.variable(3, n)

        Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1])
        R = 2 * np.diag([1, 1, 1])
        P = n * Q

        q0, u0 = self.initial_cond()

        obj = self.objective_func(q, u, self.q_goal, Q, R, P)

        opti.minimize(obj)

        opti.subject_to(self.constraints(q, u))
        #print("q", q, "q0", q0)
        opti.set_initial(q, q0)
        opti.set_initial(u, u0)

        ###### CONSTRUCT SOLVER AND SOLVE ######

        opti.solver('ipopt')
        p_opts = {"expand": False}
        s_opts = {"max_iter": 1e4, "acceptable_tol": 1e100} #, "constr_viol_tol": 1e10, "tol": 1e10, 
                    #"acceptable_obj_change_tol": 1e20, "compl_inf_tol": 1e-1,  "compl_inf_tol": 1e-1} #try changing acceptable_tol if you keep getting an infeasable problem


        opti.solver('ipopt', p_opts, s_opts)
        sol = opti.solve()

        plan = sol.value(q)
        inputs = sol.value(u)
        return plan, inputs

    def plot(self, plan, inputs):

        times = np.arange(0.0, (self.n + 1) * self.dt, self.dt)

        # Trajectory plot
        ax = plt.subplot(1, 1, 1)
        ax.set_aspect(1)
        ax.set_xlim(self.q_lb[2], self.q_ub[2])
        ax.set_ylim(self.q_lb[3], self.q_ub[3])

        for obs in self.obs_list:
            xc, yc, r = obs
            circle = plt.Circle((xc, yc), r, color='black')
            ax.add_artist(circle)

        plan_x = plan[2, :]
        plan_y = plan[3, :]
        ax.plot(plan_x, plan_y, color='green')

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")

        plt.show()

        # States plot
        plt.plot(times, plan[0, :], label='theta')
        plt.plot(times, plan[1, :], label='sigma')
        plt.plot(times, plan[2, :], label='x')
        plt.plot(times, plan[3, :], label='y')
        plt.plot(times, plan[4, :], label='theta_dot')
        plt.plot(times, plan[5, :], label='psi')
        plt.plot(times, plan[6, :], label='psi_dot')
        plt.plot(times, plan[7, :], label='v')

        plt.xlabel('Time (s)')
        plt.legend()
        plt.show()

        # Inputs plot
        plt.plot(times[:-1], inputs[0, :], label='v_dot')
        plt.plot(times[:-1], inputs[1, :], label='sigma_dot')
        plt.plot(times[:-1], inputs[2, :], label='alpha_ddot')
        #plt.plot(times[:-1], inputs[3, :], label='alpha_ddot')

        #plt.plot(times[:-1], inputs[4, :], label='alpha_ddot')
        
        plt.xlabel('Time (s)')
        plt.legend()
        plt.show()


def main():
    
    ###### PROBLEM PARAMS ######

    n = 1000
    dt = 0.01

    xy_low = [-5, -1]
    xy_high = [10, 10]
    theta_max = 1
    u_max = 10
    obs_list = [] #[[5, 5, 1]]
    q_start = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    q_goal = np.array([0, 0, 6, 5, 0, 0, 0, 0]) 

    ###### SETUP PROBLEM ######
    
    q_lb = [-theta_max, - theta_max] + xy_low + [-100, -100, -100, -100]
    q_ub = [theta_max, theta_max] + xy_high + [100, 100, 100, 100]

    u_lb = [-10, -u_max, -u_max, -u_max]
    u_ub = [10, u_max, u_max, u_max]

    ###### CONSTRUCT SOLVER AND SOLVE ######    
    dynamics = Bicycle()

    path_planner = PathPlanner(dynamics, q_start, q_goal, q_lb, q_ub, u_lb, u_ub, obs_list, n=n, dt=dt)

    plan, inputs = path_planner.plan_to_pose()

    ###### PLOT ######
    print("Final Position:", plan[:6, -1])
    path_planner.plot(plan, inputs)

if __name__ == '__main__':
    main()