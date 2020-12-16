"""
Extended kalman filter (EKF)

Author: Iñaki Lorente

"""

import math
import cmath
import numpy as np
import sympy


class EKF():
    
    def __init__(self, dt, wheelbase):
        
        # Dimensions
        dim_x = 3
        dim_z = 2
        
        #######################
        # 0. INIT
        #######################
        self.dt = dt
        self.wheelbase = wheelbase
        
        self.x = np.zeros((dim_x,1))        # True trajectory state
        self.P = np.eye(dim_x)              # uncertainty covariance
        self.G = np.eye(dim_x)              # state transition matrix
        self.R = np.eye(dim_z)              # state uncertainty
        self.Q = np.eye(dim_x)              # process uncertainty
        
        self.K = np.zeros(self.x.shape)     # kalman gain
        self.S = np.zeros((dim_z, dim_z))   # system uncertainty
        self.y = np.zeros((dim_z, 1))       # residual
        self.I = np.eye(dim_x)              # identity matrix
        
        self.x_est = self.x                 # Estimated trajectory state
        
        ##################################
        # 1. FORUMLAS: Get sympy symbols
        ##################################
        x, y, theta, v, a, w, t, lx, ly = sympy.symbols('x, y, theta, v, a, w, t, lx, ly')

        # Get wheel information
        dist = v*t # distance
        beta = (dist/w)*sympy.tan(a) # (distance/wheelbase) * steering angle
        r  = w/sympy.tan(a) # radius

        ###################
        # 1.1 System model
        ###################

        # state = [x,y,theta] (pos_x, pos_y, yaw_rate)
        self.state_formula = sympy.Matrix([x,y,theta])

        # motion = [v,a] (velocity, steering angle)
        self.motion_formula = sympy.Matrix([v,a])

        # g = Non-linear function over the state vector at time t (predicted state)
        self.g_formula = sympy.Matrix([[x-beta*sympy.sin(theta)+r*sympy.sin(theta + beta)],
                          [y+r*sympy.cos(theta)-r*sympy.cos(theta + beta)],
                          [theta+beta]
                         ])

        # Jacobian of g evaluated using state
        self.G_formula = self.g_formula.jacobian(self.state_formula)

        # Jacobian of g evaluated using motion
        self.V_formula = self.g_formula.jacobian(self.motion_formula)
        
        ########################
        # 1.2 Measurement model
        ########################

        # It depends on the measurement sensor used
        # this case sensor provides noise bearing and range
        # based on landmarks

        # range
        r = sympy.sqrt((lx-x)**2 + (ly-y)**2)
        # bearing
        b = sympy.atan((ly-y)/(lx-x))

        # h = Non-linear function over the state vector at time t (predicted mesurement)
        self.h_landmark_formula = sympy.Matrix([[r],
                                                [b-theta]
                                               ])
        
        self.h_odom_formula = sympy.Matrix([[x],
                                            [y],
                                            [theta]
                                           ])

        # Jacobian of h evaluated using state
        self.H_landmark_formula = self.h_landmark_formula.jacobian(self.state_formula)
        
        self.H_odom_formula = self.h_odom_formula.jacobian(self.state_formula)

        ##################################
        # 2. Save values in a dictionary
        ##################################
        self.ekf_dict = {x: 0, y: 0, theta:0,
                         v:0, a:0, 
                         t:dt, w:wheelbase}
        
        # This just creates a link to the dictionay keys
        # the value of the following variables are just
        # names (dictionary key names)
        self.x_x, self.x_y, self.theta = x, y, theta 
        self.v, self.a = v, a
        
    def __repr__(self):
        return '\n'.join([
            'Extended Kalman Filter object',
            '------------------------------',
            'x: ', str(self.x),
            'P: ', str(self.P),
            'x_est: ', str(self.x_est),
            'G: ', str(self.G),
            'Q: ', str(self.Q),
            'R: ', str(self.R),
            'K: ', str(self.K),
            'y: ', str(self.y),
            'S: ', str(self.S)
            ])
        
    def predict(self, x, u):
        '''EKF Prediction step
                input  -> u = motion control [v,a] 
                call   -> move function
                action -> get P based on new values
        '''      
        
        self.x = x

        # Write values on EKF dictionary
        self.ekf_dict[self.x_x] = self.x[0]
        self.ekf_dict[self.x_y] = self.x[1]
        self.ekf_dict[self.theta] = self.x[2]
        self.ekf_dict[self.v] = u[0]
        self.ekf_dict[self.a] = u[1]
        
        # Get G,V values applying dicctionary values to formulas
        self.G = np.array(self.G_formula.evalf(subs=self.ekf_dict)).astype(float)
        self.V = np.array(self.V_formula.evalf(subs=self.ekf_dict)).astype(float)

        # M = Covariance of motion noise
        self.M = np.array([[u[0]**2, 0], 
                           [0, u[1]**2]
                          ])

        self.Q = np.dot(self.V, self.M).dot(self.V.T)
        
        # P = Error covariance
        self.P = np.dot(self.G, self.P).dot(self.G.T) + self.Q
        

    def update_landmark_method(self, landmark, std_range, std_bearing):
        '''EKF Update step based on landmarks
                input  -> landmarks, sensor range and bearing          
                action -> update P and state (x) based on measurements
        '''
        for lmark in landmark:
            hyp = (lmark[0] - self.x[0])**2 + (lmark[1] - self.x[1])**2
            dist = math.sqrt(hyp)

            # Get measurement h on last current state
            # This state (x) has predictions applied
            self.h = np.array([[dist],
                               [math.atan2(lmark[1] - self.x[1], lmark[0] - self.x[0]) - self.x[2]]
                              ])

            # Get jacobian of h on last current state
            # This state (x) has predictions applied
            self.H = np.array([[-(lmark[0] - self.x[0]) / dist, -(lmark[1] - self.x[1]) / dist, 0],
                               [(lmark[1] - self.x[1]) / hyp,  -(lmark[0] - self.x[0]) / hyp, -1]
                              ])           

            # Get measurement based on true state (x)
            # This state (x) is prior to predictions
            x, y = self.x[0], self.x[1]
            d = math.sqrt((lmark[0] - x)**2 + (lmark[1] - y)**2)  
            a = math.atan2(lmark[1] - y, lmark[0] - x) - self.x[2]
            self.z = np.array([[d + np.random.randn()*std_range],
                               [a + np.random.randn()*std_bearing]
                              ])

            # Covariance of measurement noise
            self.R = np.diag([std_range**2, std_bearing**2])

            # K = Kalman Gain
            PHT = np.dot(self.P, self.H.T)
            self.S = np.dot(self.H,PHT) + self.R
            self.K = PHT.dot(np.linalg.inv(self.S))

            # Update the estimate via mesurement (state)
            self.y = self.residual(self.z, self.h)
            self.x_est = self.x + np.dot(self.K, self.y)

            # Update Error covariance (P)
            self.P = (self.I - np.dot(self.K,self.H)).dot(self.P)
            #I_KH = self.I - np.dot(self.K, self.H)
            #self.P = np.dot(I_KH, self.P).dot(I_KH.T) + np.dot(self.K, self.R).dot(self.K.T)
            
        return self.x_est, self.P
    
    def update_odom_method(self, odom):
        '''EKF Update step based on odometry measurement
                input  -> [x,y,theta]         
                action -> update P and state (x) based on measurements
        '''
        
        # Get h,H values applying dicctionary values to formulas
        self.h = np.array(self.h_odom_formula.evalf(subs=self.ekf_dict)).astype(float)
        self.H = np.array(self.H_odom_formula.evalf(subs=self.ekf_dict)).astype(float)        

        # Get measurement based on true state (x)
        Qsim = np.diag([0.5, 0.5, 0.5])**2
        Rsim = np.diag([1.0, 1.0, np.deg2rad(30.0)])**2
        self.z = np.array([[self.x[0] + np.random.randn() * Qsim[0, 0]],
                           [self.x[1] + np.random.randn() * Qsim[1, 1]],
                           [self.x[2] + np.random.randn() * Qsim[2, 2]],
                          ])

        # Covariance of measurement noise
        self.R = np.diag([odom[0]**2, odom[1]**2, odom[2]**2])

        # K = Kalman Gain
        PHT = np.dot(self.P, self.H.T)
        self.S = np.dot(self.H,PHT) + self.R
        self.K = PHT.dot(np.linalg.inv(self.S))

        # Update the estimate via mesurement (state)
        self.y = self.residual(self.z, self.h)
        self.x_est = self.x + np.dot(self.K, self.y)

        # Update Error covariance (P)
        self.P = (self.I - np.dot(self.K,self.H)).dot(self.P)
        #I_KH = self.I - np.dot(self.K, self.H)
        #self.P = np.dot(I_KH, self.P).dot(I_KH.T) + np.dot(self.K, self.R).dot(self.K.T)
            
        return self.x_est, self.P

            
    def residual(self, a, b):
        """ compute residual (a-b) between measurements containing 
        [range, bearing]. Bearing is normalized to [-pi, pi)"""
        y = a - b
        y[1,0] = y[1,0] % (2 * np.pi)    # force in range [0, 2 pi)
        if y[1,0] > np.pi:               # move to [-pi, pi)
            y[1,0] -= 2 * np.pi
        return y

    def move(self, x, u, dt):
        '''SIM ROBOT MOVEMENT. To be used if we don't have odometry 
           from robot. This function is mainly used for 
           test purposes.
        '''
        theta = x[2] # x = [x, y, theta] where 'theta' is the steer angle
        vel = u[0]      # u = [v, a] -> get velocity
        steering_angle = u[1] # u = [v, a] -> get steer angle
        dist = vel * dt # distance

        if abs(steering_angle) > 0.001: # is robot turning?
            beta = (dist / self.wheelbase) * math.tan(steering_angle)
            r = self.wheelbase / math.tan(steering_angle) # radius

            dx = np.array([[-r*math.sin(theta) + r*math.sin(theta + beta)], 
                           [r*math.cos(theta) - r*math.cos(theta + beta)], 
                           [beta]])
        else: # moving in straight line
            dx = np.array([[dist*cos(theta)], 
                           [dist*sin(theta)], 
                           [theta]])
            
        # Update current state
        self.x = x + dx
        
        return self.x
    
    def plot_covariance_ellipse(self, xEst, PEst):
        ''' Just a way to plot P covariance
        based on https://github.com/AtsushiSakai/PythonRobotics
        '''
        Pxy = PEst[0:2, 0:2]
        eigval, eigvec = np.linalg.eig(Pxy)

        if eigval[0] >= eigval[1]:
            bigind = 0
            smallind = 1
        else:
            bigind = 1
            smallind = 0

        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        a = cmath.sqrt(eigval[bigind])
        b = cmath.sqrt(eigval[smallind])
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
        R = np.array([[math.cos(angle), math.sin(angle)],
                      [-math.sin(angle), math.cos(angle)]])
        fx = R.dot(np.array([[x, y]]))
        px = np.array(fx[0, :] + xEst[0, 0]).flatten()
        py = np.array(fx[1, :] + xEst[1, 0]).flatten()

        #plt.plot(px, py, "--g")

        return px, py