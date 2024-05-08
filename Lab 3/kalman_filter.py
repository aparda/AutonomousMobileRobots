import numpy as np


# Part 3: Comment the code explaining each part
class kalman_filter:
    
    # Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P=P
        self.Q=Q
        self.R=R
        self.x=x
        self.dt =dt
        
    # Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = self.jacobian_A() # we want to use the jacobian here for the prediction step

        # we want to use the Jacobian for the update step
        self.C = self.jacobian_H()

        #updates the value of x
        self.motion_model()
        
        # calculate the predicted covatiance estimate
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):

        # the "innovation or resiudal" covariance which represents the
        # discrepencies between the predicted and actual measurement
        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R

        # trying to get the "near optimal" kalman gain
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))


        # using surprise error rather than the error residuals from measurement
        surprise_error= z - self.measurement_model()


        # update the state estimate to account for the surprise error
        self.x=self.x + np.dot(kalman_gain, surprise_error)

        # update our covariance estimate (I - KH)P
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax
            v*w, # ay
        ])
        
    # Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])



    # Part 3: Fill in the ... of the A Jacobian
    # we need this because this is a non-linear system
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              -v*np.sin(th)*dt, 0,         np.cos(th)*dt,  0],
            [0, 1,              v*np.cos(th)*dt, 0,          np.sin(th)*dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    

    # Part 3: Implement here the jacobian of the H matrix (measurements)
    # we need this because this is a non-linear system   
    def jacobian_H(self):
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # Part 3: return the states here    
    def get_states(self):
        #"kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y"
        return {"kf_ax" : self.x[-1], "kf_ay":self.x[3]*self.x[4], "kf_vx": self.x[4], "kf_w": self.x[3],
                 "kf_x": self.x[0], "kf_y":self.x[1], "theta": self.x[2]}
