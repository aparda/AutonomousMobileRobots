import numpy as np


from pid import PID_ctrl
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error

M_PI=3.1415926535

P=0; PD=1; PI=2; PID=3

class controller:

    # Default gains of the controller for linear and angular motions
    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2):
        
        # Part 5 and 6: Modify the below lines to test your PD, PI, and PID controller
        # the values get passed when initializing the controller
        self.PID_linear=PID_ctrl(P, klp, klv, kli, filename_="linear.csv")
        self.PID_angular=PID_ctrl(PID, kap, kav, kai, filename_="angular.csv")

    
    def vel_request(self, pose, goal, status):
        
        e_lin=calculate_linear_error(pose, goal)
        e_ang=calculate_angular_error(pose, goal)

        # picking this as our convergence criterion (for now)
        # modified this to be more a curvy path, rather than point stop and go to avoid jerky motins
        if abs(e_ang) < 0.025:
            e_ang = 0
        if abs(e_lin) < 0.025:
            e_lin = 0

        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status)

        # Part 4: Add saturation limits for the robot linear and angular velocity
        #https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html?highlight=ANGULAR
        linear_vel = (0.306*linear_vel/abs(linear_vel)) if abs(linear_vel) > 0.306 else linear_vel
        angular_vel= (1.9*angular_vel/abs(angular_vel)) if abs(angular_vel) > 1.9 else angular_vel
        return linear_vel, angular_vel
    

class trajectoryController(controller):

    def __init__(self, klp=0.2, klv=0.2, kli=0.2, kap=0.2, kav=0.2, kai=0.2, lookAhead=1.0):
        
        super().__init__(klp, klv, kli, kap, kav, kai)
        self.lookAhead=lookAhead
    
    def vel_request(self, pose, listGoals, status):
        
        goal=self.lookFarFor(pose, listGoals)
        
        finalGoal=listGoals[-1]
        e_lin=calculate_linear_error(pose, finalGoal)
        e_ang=calculate_angular_error(pose, goal, True)

        linear_vel=self.PID_linear.update([e_lin, pose[3]], status)
        angular_vel=self.PID_angular.update([e_ang, pose[3]], status) 

        # Part 4: Add saturation limits for the robot linear and angular velocity
        #https://turtlebot.github.io/turtlebot4-user-manual/overview/features.html?highlight=ANGULAR
        linear_vel = 0.306*linear_vel/abs(linear_vel) if abs(linear_vel) > 0.306 else linear_vel
        angular_vel= 1.90*angular_vel/abs(angular_vel) if abs(angular_vel) > 1.90 else angular_vel
        
        return linear_vel, angular_vel

    def lookFarFor(self, pose, listGoals):
        
        poseArray=np.array([pose[0], pose[1]]) 
        listGoalsArray=np.array(listGoals)

        distanceSquared=np.sum((listGoalsArray-poseArray)**2,
                               axis=1)
        closestIndex=np.argmin(distanceSquared)

        return listGoals[ min(closestIndex + 3, len(listGoals) - 1) ]
