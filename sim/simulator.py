#python dependencies
from re import I
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
from collections import deque

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
        self.inputs = [np.zeros((6, 1))] #(control_dimn x N) vector of control inputs over time
        self.times = None

        print("self.inputs:", self.inputs)

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
        times = np.arange(0,T,self.dt) #create an array of times to go through
        self.times = times #store in class variable

        
        #print("self.inputs second print",self.inputs)

        for t in times:
            u_t = self.controller.control_input(q_t, q_goal) #get the current input
            self.inputs.append(u_t) #begin populating the input vector

            #Simulate the dynamics by getting the next step
            q_t = self.dynamics.q_tp1(q_t, u_t, t, 0.1) #get the next state by calling q_tp1 dynamics method
            #add the state and the input to the object parameters
            self.states.append(q_t)

        return self.states, self.inputs, times

    def plot_results(self, plot_args = [True, False, True]):
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
            self.states = np.asarray(self.states)
            for i in range(6):
                axs[i].plot(self.times, self.states[1:,i,0])
                axs[i].set(xlabel=xlabels, ylabel=ylabels[i]) #pull labels from the list above
            plt.show()

        if plot_args[1]:
            #next, plot the input variables in a 5x1 subplot
            fig, axs = plt.subplots(5)
            fig.suptitle('Evolution of Bicycle Inputs in Time')
            ylabels = ['v', 'v_dot', 'sigma', 'sigma_dot', 'alpha_ddot']
            self.inputs = np.asarray(self.inputs)

            for i in range(5):
                axs[i].plot(self.times, self.inputs[1:])
                axs[i].set(xlabel=xlabels, ylabel=ylabels[i])
            plt.show()

        if plot_args[2]:
            #next, plot the x and y against each other to show path in space
            plt.plot(self.states[1:, 2], self.states[1:, 3])
            plt.title("XY Path of bicycle rear wheel")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.show()

    def animate(self, plot_args = [True, True]):
        """
        Runs two animations of the bicycle
        1) Lateral dynamics from above in XY frame
        2) Balancing dynamics in frame traveling with bicycle
        """
        t_stop = 10  # how many seconds to simulate
        history_len = 200  # how many trajectory points to display
        self.states = np.asarray(self.states)
        b = 2
        a = 1

        # create a time array from 0..t_stop sampled at dt second steps
        dt = self.dt
        t = np.arange(0, t_stop, dt)

        if plot_args[0]:
            #Lateral dynamics

            x1_test = [np.sin(angle) for angle in np.arange(0, 10, dt)]
            y1_test = [y for y in np.arange(0, 10, dt)]

            x2_test = [x + 5 for x in x1_test]
            y2_test = [y + b  for y in np.arange(0, 10, dt)]

            x1 = 5*x1_test
            y1 = 5*y1_test

            x2 = 5*x2_test
            y2 = 5*y2_test

            fig = plt.figure(figsize=(5, 4))
            #maybe integrate with the map generation code from RRT over here
            ax = fig.add_subplot(autoscale_on=False, xlim=(-10, 10), ylim=(-10, 10))
            ax.set_aspect('equal')
            ax.grid()

            

            line, = ax.plot([],[])
            backwheel, = ax.plot([],[],lw = 3,color = 'r' )
            frontwheel, = ax.plot([],[],lw = 3, color = 'r')
            trace, = ax.plot([], [], '.-', lw=1, ms=2)
            time_template = 'time = %.1fs'
            time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
            history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)



            def animate(i):


                x1_test = [np.sin(angle) for angle in np.arange(0, 10, dt)]
                y1_test = [y for y in np.arange(0, 10, dt)]

                x2_test = [x + 1 for x in x1_test]
                y2_test = [y + b  for y in np.arange(0, 10, dt)]

                x1 = 5*x1_test
                y1 = 5*y1_test

                x2 = 5*x2_test
                y2 = 5*y2_test




                #NEED TO CHANGE THIS TO ACC GET FROM A STATE
                thisx = [x1[i], x2[i]]
                thisy = [y1[i], y2[i]]


                
                def get_back_wheel(length):
                    x1 = thisx[0]
                    y1 = thisy[0]

                    x2 = thisx[1]
                    y2 = thisy[1]
                    #print(x1,y1,x2,y2)


                    back_xs = [x1-((length/2)*(x2-x1)),x1+((length/2)*(x2-x1))]
                    back_ys = [y1-((length/2)*(y2-y1)),y1+((length/2)*(y2-y1))]
                    return back_xs,back_ys

                def get_front_wheel(length,sigma):
                    x1 = thisx[0]
                    y1 = thisy[0]

                    x2 = thisx[1]
                    y2 = thisy[1]
                    #print(x1,y1,x2,y2)

                    theta = np.arctan(y2-y1/x2-x1)
                    #print(y2-y1,x2-x1)

                    topx = x2 + (length/2 * np.cos(theta + sigma))
                    bottomx =  x2 - (length/2 * np.cos(theta + sigma)) 

                    topy = y2 + (length/2 * np.sin(theta + sigma))
                    bottomy =  y2 - (length/2 * np.sin(theta + sigma)) 

                    front_xs = [bottomx,topx]
                    front_ys = [bottomy,topy]

                    return front_xs,front_ys



                if i == 0:
                    history_x.clear()
                    history_y.clear()

                history_x.appendleft(thisx[0])
                history_y.appendleft(thisy[0])

                line.set_data(thisx, thisy)
                bx_s , by_s = get_back_wheel(0.5)
                backwheel.set_data(bx_s,by_s)
                fx_s , fy_s = get_front_wheel(0.5,np.pi/4)
                frontwheel.set_data(fx_s,fy_s)
                trace.set_data(history_x, history_y)
                time_text.set_text(time_template % (i*dt))
                return line, trace, time_text,backwheel,frontwheel


            ani = animation.FuncAnimation(
                fig, animate, len(self.times), interval=dt*1000, blit=True)
            plt.show()
        
        if plot_args[1]:
            #Balancing dynamics

            t_stop = 10  # how many seconds to simulate
            history_len = 50  # how many trajectory points to display
            self.states = np.asarray(self.states)

            # create a time array from 0..t_stop sampled at 0.02 second steps
            dt = self.dt
            t = np.arange(0, t_stop, dt)
            psi_test = [np.sin(angle) for angle in np.arange(0, 10, dt)]
            

            x1 = a*np.sin(psi_test)#np.pi/4*np.ones(len(self.times)))#self.states[1:,4,0]#self.times #self.states[1:,2,0]
            y1 = a*np.cos(psi_test)#np.pi/4*np.ones(len(self.times))#self.times #self.states[1:,2,0]

            fig = plt.figure(figsize=(5, 4))
            ax = fig.add_subplot(autoscale_on=False, xlim=(-5, 5), ylim=(-5, 5))
            ax.set_aspect('equal')
            ax.grid()

            line, = ax.plot([], [], 'o-', lw=2)
            trace, = ax.plot([], [], '.-', lw=1, ms=2)
            time_template = 'time = %.1fs'
            time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
            history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)


            def animate(i):
                #print(x1[i])
                thisx = [0, x1[i]]
                thisy = [0, y1[i]]

                if i == 0:
                    history_x.clear()
                    history_y.clear()

                history_x.appendleft(thisx[1])
                history_y.appendleft(thisy[1])

                line.set_data(thisx, thisy)
                trace.set_data(history_x, history_y)
                time_text.set_text(time_template % (i*dt))
                return line, trace, time_text


            ani = animation.FuncAnimation(
                fig, animate, len(self.times), interval=dt*1000, blit=True)
            plt.show()





    def animate_plan(self, plan, inputs):
        """
        Runs ONE animation of the bicycle
        1) Lateral dynamicS from above in XY frame
        
        """
        t_stop = 10  # how many seconds to simulate
        history_len = 200  # how many trajectory points to display
        self.states = np.asarray(self.states)
        # create a time array from 0..t_stop sampled at dt second steps
        dt = self.dt

        
        #Lateral dynamics
        fig = plt.figure(figsize=(5, 4))
        #maybe integrate with the map generation code from RRT over here
        ax = fig.add_subplot(autoscale_on=False, xlim=(-10, 10), ylim=(-10, 10))
        ax.set_aspect('equal')
        ax.grid()

        line, = ax.plot([],[])
        backwheel, = ax.plot([],[],lw = 3,color = 'r' )
        frontwheel, = ax.plot([],[],lw = 3, color = 'r')
        trace, = ax.plot([], [], '.-', lw=1, ms=2)
        time_template = 'time = %.1fs'
        time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
        history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)



        def animate(i):

            #Plots the bicylce fram
            #MAKE THIS INTO A FUNC OF THETA
            theta = plan[0,i]
            length = self.dynamics.b
            sigma = inputs[2,i]
            thisx = [plan[2, i], plan[2, i] + length*np.cos(theta)]
            thisy = [plan[3, i], plan[3, i] + length*np.sin(theta)]


            
            def get_back_wheel(length):
                x1 = thisx[0]
                y1 = thisy[0]

                x2 = thisx[1]
                y2 = thisy[1]

                back_xs = [x1-((length/2)*(x2-x1)),x1+((length/2)*(x2-x1))]
                back_ys = [y1-((length/2)*(y2-y1)),y1+((length/2)*(y2-y1))]
                return back_xs,back_ys

            def get_front_wheel(length,sigma):
                x1 = thisx[0]
                y1 = thisy[0]

                x2 = thisx[1]
                y2 = thisy[1]

                theta = np.arctan(y2-y1/x2-x1)

                topx = x2 + (length/2 * np.cos(theta + sigma))
                bottomx =  x2 - (length/2 * np.cos(theta + sigma)) 

                topy = y2 + (length/2 * np.sin(theta + sigma))
                bottomy =  y2 - (length/2 * np.sin(theta + sigma)) 

                front_xs = [bottomx,topx]
                front_ys = [bottomy,topy]

                return front_xs,front_ys



            if i == 0:
                history_x.clear()
                history_y.clear()

            history_x.appendleft(thisx[0])
            history_y.appendleft(thisy[0])

            line.set_data(thisx, thisy)
            bx_s , by_s = get_back_wheel(0.5)
            backwheel.set_data(bx_s,by_s)
            #TAKE ANGLE FROM PLAN 
            print("sigma:",sigma)
            fx_s , fy_s = get_front_wheel(0.5,sigma)
            frontwheel.set_data(fx_s,fy_s)
            trace.set_data(history_x, history_y)
            time_text.set_text(time_template % (i*dt))
            return line, trace, time_text,backwheel,frontwheel


        ani = animation.FuncAnimation(
            fig, animate, len(self.times), interval=dt*1000, blit=True)
        plt.show()



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

