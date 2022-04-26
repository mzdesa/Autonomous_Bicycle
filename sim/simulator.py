#python dependencies
from ast import BitXor
from re import I
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
from collections import deque
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d

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
            sigma = plan[1,i]
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

            def get_front_wheel(length, sigma, theta):
                x2 = thisx[1]
                y2 = thisy[1]

                topx = x2 + (length/2 * np.cos(sigma+ theta))
                bottomx =  x2 - (length/2 * np.cos(sigma+ theta)) 

                topy = y2 + (length/2 * np.sin(sigma+ theta))
                bottomy =  y2 - (length/2 * np.sin(sigma+ theta)) 

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
            #print("sigma:",sigma)
            fx_s , fy_s = get_front_wheel(0.5,sigma, theta)
            frontwheel.set_data(fx_s,fy_s)
            trace.set_data(history_x, history_y)
            time_text.set_text(time_template % (i*dt))
            return line, trace, time_text,backwheel,frontwheel


        ani = animation.FuncAnimation(
            fig, animate, len(self.times), interval=dt*1000, blit=True)
        plt.show()






        #Balancing dynamics

        t_stop = 10  # how many seconds to simulate
        history_len = 50  # how many trajectory points to display
        self.states = np.asarray(self.states)

        # create a time array from 0..t_stop sampled at 0.02 second steps
        dt = self.dt
        t = np.arange(0, t_stop, dt)
        
        fig = plt.figure(figsize=(5, 4))
        ax = fig.add_subplot(autoscale_on=False, xlim=(-1, 1), ylim=(-1, 1))
        ax.set_aspect('equal')
        ax.grid()

        line, = ax.plot([], [], 'o-', lw=2)
        trace, = ax.plot([], [], '.-', lw=1, ms=2)
        time_template = 'time = %.1fs'
        time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
        history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)


        def animate(i):
            #print(x1[i])
            psi = plan[5, i]
            x1 = self.dynamics.a * np.sin(psi)
            y1 = self.dynamics.a * np.cos(psi)
            thisx = [0, x1]
            thisy = [0, y1]

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








    







    def animate_plan_3D(self, plan, inputs):
        """
        Runs ONE animation of the bicycle
        1) Lateral dynamicS from above in XY frame
        
        """

        #Functions for tranformation of circle objects
        
        def rotation_matrix(d):
            """
            Calculates a rotation matrix given a vector d. The direction of d
            corresponds to the rotation axis. The length of d corresponds to 
            the sin of the angle of rotation.

            Variant of: http://mail.scipy.org/pipermail/numpy-discussion/2009-March/040806.html
            """
            sin_angle = np.linalg.norm(d)

            if sin_angle == 0:
                return np.identity(3)

            d /= sin_angle

            eye = np.eye(3)
            ddt = np.outer(d, d)
            skew = np.array([[    0,  d[2],  -d[1]],
                        [-d[2],     0,  d[0]],
                        [d[1], -d[0],    0]], dtype=np.float64)

            M = ddt + np.sqrt(1 - sin_angle**2) * (eye - ddt) + sin_angle * skew
            return M



        def pathpatch_2d_to_3d(pathpatch, z = 0, normal = 'z'):
            """
            Transforms a 2D Patch to a 3D patch using the given normal vector.

            The patch is projected into they XY plane, rotated about the origin
            and finally translated by z.
            """
            if type(normal) is str: #Translate strings to normal vectors
                index = "xyz".index(normal)
                normal = np.roll((1.0,0,0), index)

            normal /= np.linalg.norm(normal) #Make sure the vector is normalised

            path = pathpatch.get_path() #Get the path and the associated transform
            trans = pathpatch.get_patch_transform()

            path = trans.transform_path(path) #Apply the transform

            pathpatch.__class__ = art3d.PathPatch3D #Change the class
            pathpatch._code3d = path.codes #Copy the codes
            pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    

            verts = path.vertices #Get the vertices in 2D

            d = np.cross(normal, (0, 0, 1)) #Obtain the rotation vector    
            M = rotation_matrix(d) #Get the rotation matrix

            pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])


        def pathpatch_translate(pathpatch, delta):
            """
            Translates the 3D pathpatch by the amount delta.
            """
            pathpatch._segment3d += delta


        history_len = 200  # how many trajectory points to display
        self.states = np.asarray(self.states)
        dt = self.dt

        
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        line, = ax.plot([],[],[], color="brown")
        line2, = ax.plot([],[],[], color="brown")
        line3, = ax.plot([],[],[], color="brown")
        trace, = ax.plot([], [], [],'.-', lw=1, ms=2, color = 'orange')
        time_template = 'time = %.1fs'
        time_text = ax.text(0.05, 0.9, 5, '', transform=ax.transAxes)
        history_x, history_y = deque(maxlen=history_len), deque(maxlen=history_len)

        def animate(i):
            [p.remove() for p in ax.patches]
            theta = plan[0,i]
            length = self.dynamics.b
            sigma = plan[1,i]
            thisx = [plan[2, i], plan[2, i] + length*np.cos(theta)]
            thisy = [plan[3, i], plan[3, i] + length*np.sin(theta)]


            
            def get_back_wheel():
                return thisx[0], thisy[0]

            def get_front_wheel():
                return thisx[1],thisy[1]

            if i == 0:
                history_x.clear()
                history_y.clear()

            history_x.appendleft(thisx[0])
            history_y.appendleft(thisy[0])

            bx_s , by_s = get_back_wheel()
            fx_s , fy_s = get_front_wheel()

            line.set_data(np.asarray([bx_s, fx_s]), np.asarray([by_s, fy_s]))
            line.set_3d_properties([0, 0.7])

            line2.set_data(np.asarray([fx_s, fx_s]), np.asarray([fy_s, fy_s]))
            line2.set_3d_properties([0.7, 0])


            line3.set_data(np.asarray([fx_s - 0.3 * np.sin(sigma + theta), fx_s + 0.3 * np.sin(sigma + theta)]),
                            np.asarray([fy_s + 0.3 * np.cos(sigma + theta), fy_s - 0.3 * np.cos(sigma + theta)]))
            line3.set_3d_properties(0.7)
            

            trace.set_data(np.asarray(history_x), np.asarray(history_y))
            trace.set_3d_properties(0)
            time_text.set_text(time_template % (i*dt))

            backwheel_3D = Circle((0, 0), 0.4, edgecolor="black")
            ax.add_patch(backwheel_3D)
            pathpatch_2d_to_3d(backwheel_3D, z = 0, normal = [-np.sin(theta), np.cos(theta), 0])
            pathpatch_translate(backwheel_3D, [bx_s, by_s, 0])

            frontwheel_3D = Circle((0, 0), 0.4, edgecolor="black")
            ax.add_patch(frontwheel_3D)
            pathpatch_2d_to_3d(frontwheel_3D, z = 0, normal = [-np.sin(theta+sigma), np.cos(theta+sigma), 0])
            pathpatch_translate(frontwheel_3D, [fx_s, fy_s, 0])
     
            
            return line, line2, line3, trace

        

        # Setting the axes properties
        ax.set_xlim3d([-1.0, 10.0])
        ax.set_xlabel('X')

        ax.set_ylim3d([-1.0, 10.0])
        ax.set_ylabel('Y')

        ax.set_zlim3d([0.0, 10.0])
        ax.set_zlabel('Z')
        

        ani = animation.FuncAnimation(
            fig, animate, len(self.times), interval=dt*1000, blit=False)

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

