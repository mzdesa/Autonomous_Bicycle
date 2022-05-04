
import numpy as np
import matplotlib.pyplot as plt
from simulator import *
from controllers import *
from dynamics import *
from path_planner import *
from Map import *
from obstacles import *
#EKF for bicycle


class EKF:
    """EKF class - Defines an Extended Kalman Filter to run in a loop for every new measurement"""
    
    def __init__(self, length = 1):
        #Bicycle length
        self.b = length
        #Process noise variance
        self.Sigma_vv = np.matrix([[0.01, 0, 0], 
                                    [0, 0.01, 0],
                                    [0, 0, 0.01]])
        #Measurement noise variance
        self.Sigma_ww = np.matrix([0.01])


    def prediction(self, xm, Pm, u, dt):
        """Predicts the next state based on an input u and previus estimate of state and variance, xm and Pm"""
        
        #1
        #inputs = [sigma,ax, ay, ax]
        #states = [x, y, theta, v, psi]
        """sigma = u[0]
        ax = u[1]
        ay = u[2]
        az = u[3]
        
        x = xm[0, 0]
        y = xm[1, 0]
        theta = xm[2, 0]
        v = xm[3, 0]
        psi = xm[4, 0]

        q_dot = np.matrix([[v*np.cos(theta)], 
                    [v*np.sin(theta)],
                    [v*np.tan(sigma)/self.b],
                    [ax], 
                    [np.arctan((ay/az)]])

        A = np.eye(5) + dt*np.matrix([[0, 0, -v*np.sin(theta), np.cos(theta), 0], 
                                    [0, 0, v*np.cos(theta), np.sin(theta), 0],
                                    [0, 0, 0, np.tan(sigma)/self.b, 0],
                                    [0, 0, 0, 0, 0] 
                                    [0, 0, 0, 0, 0]])
        L = dt*np.eye(5)"""



        #2
        #inputs = [sigma, v]
        #states = [x, y, theta]
        sigma = u[0]
        v = u[1] 
        
        x = xm[0, 0]
        y = xm[1, 0]
        theta = xm[2, 0]

        q_dot = np.matrix([[v*np.cos(theta)], 
                    [v*np.sin(theta)],
                    [v*np.tan(sigma)/self.b]])

        A = np.eye(3) + dt*np.matrix([[0, 0, -v*np.sin(theta)], 
                                    [0, 0, v*np.cos(theta)],
                                    [0, 0, 0]])
        L = dt*np.eye(3)




        #3
        #inputs = [theta, ax, ay, az]
        #states = [x, y, sigma, v]
        """theta = u[0]
        ax = u[1]
        ay = u[2]
        az = u[3]
        
        x = xm[0, 0]
        y = xm[1, 0]
        sigma = xm[2, 0]
        v = xm[3, 0]

        q_dot = np.matrix([[v*np.cos(theta)], 
                    [v*np.sin(theta)], 
                    [np.arctan(self.b*theta/v)], 
                    [ax]])

        A = np.eye(4) + dt*np.matrix([[0, 0, 0, np.cos(theta)], 
                                    [0, 0, 0, np.sin(theta)],
                                    [0, 0, 0, 0]])
        L = dt*np.eye(4)"""


        #Prediction step equations
        Pp = A @ Pm @ A.T + L @ self.Sigma_vv @ L.T
        xp = xm + dt*q_dot

        return xp, Pp

    def update(self, xp, Pp, z):
        """Finds the posteriori estimate xm and Pm using the measurement z"""

        #2 
        # Measurement is theta
        x = xp[0, 0]
        y = xp[1, 0]
        theta = xp[2, 0]

        h = np.matrix([theta])
        H = np.matrix([0, 0, 1])

        #Measurement update equations
        K = Pp @ H.T @ np.linalg.inv(H @ Pp @ H.T + self.Sigma_ww)
        Pm = (np.eye(3) - K @ H) @ Pp
        xm = xp + K * (z - h)
        return xm, Pm

    def run_filter(self, xm, Pm, z, u, dt):
        "Runs one iteration of the EKF"

        xp, Pp = self.prediction(xm, Pm, u, dt)
        xm, Pm = self.update(xp, Pp, z)
        return xm, Pm


    def plot(self, states, inputs, zs, dt):
        """Plots the estimated states and inputs"""
        n = len(states)
        times = np.arange(0.0, dt*n, dt)
        states = np.asarray(states)
        inputs = np.asarray(inputs)

        # Trajectory plot
        ax = plt.subplot(1, 1, 1)
        x = states[:, 0]
        y = states[:, 1]
        ax.plot(x, y, color='green')

        plt.title("Trajectory")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.show()

        #States plot
        fig, ax = plt.subplots(3, 1, sharex=True)
        ax[0].plot(times, states[:, 0], label='x')
        ax[1].plot(times, states[:, 1], label='y')
        ax[2].plot(times, states[:, 2], label='theta')
        ax[2].plot(times, zs, '--', label='Measurement')

        ax[0].set_ylabel('x [m]')
        ax[1].set_ylabel('y [m]')
        ax[2].set_ylabel('$\Theta$ [rad]')

        
        plt.xlabel('Time (s)')
        plt.legend()
        plt.title("States")
        plt.show()

        # Inputs plot
        figHist, ax = plt.subplots(2, 1, sharex=True)
        ax[0].plot(times[:-1], inputs[:, 0], label='sigma')
        ax[1].plot(times[:-1], inputs[:, 1], label='v')

        ax[0].set_ylabel('sigma [rad]')
        ax[1].set_ylabel('v [m/s]')

        
        plt.xlabel('Time (s)')
        plt.legend()
        plt.title("Inputs")
        plt.show()



#Example run

#First Generate path plan
balance_controller = BalancePID(np.array([1, 1, 1]))
path_planner_controller = PathPlannerController() #For now, just try the balancing dynamics, so use the zero planning controller
bicycle_controller = VehicleController(balance_controller, path_planner_controller) #create the full state controller
#IM NOT SURE WE ACTUALLY USE THE PATH PLANNER CONTROLLER CLASS
#define dynamics
dynamics = Bicycle() #define a bicycle object
#list of [radius,x,y]s for circular obstacles.
Circular_Obstacles_coordinates = [[1,4,4],[1,6,6]]
Circular_Obstacles_list = [Circular_Obstacle(x[0],x[1],x[2]) for x in Circular_Obstacles_coordinates]
my_blank_map = Map(0,10,0,10,Circular_Obstacles_list) #define a 10x10 map with no obstacles.
#Find optimal path from a start and goal position
q_start = np.array([0, 0, 1, 1, 0, 0, 0, 0])
q_goal = np.array([0, 0, 8, 8, 0, 0, 0, 0]) 
path_planner = PathPlanner(dynamics, q_start, q_goal,my_blank_map)
plan, inputs = path_planner.plan_to_pose()



#Create a Kalman filter
kalman_filer = EKF()

#initial values
xm= np.matrix([0, 0, 0]).T
Pm= np.eye(3)
dt = 0.01

#Inputs
sigmas = plan[1, :]
vs = plan[7, :]

#Measurements
thetas = plan[0, :]

#For plotting
states = []
us = []
states.append(xm.T.tolist()[0])

#Run the filter for n iterations
n = len(thetas) -1 
for i in range(n):
    u = [sigmas[i], vs[i]]
    z = thetas[i]
    xm, Pm = kalman_filer.run_filter(xm, Pm, z, u, dt)

    #For plotting
    states.append(xm.T.tolist()[0])
    us.append([sigmas[i], vs[i]])

#Plot estimate
measurements = thetas
kalman_filer.plot(states, us, measurements, dt)





