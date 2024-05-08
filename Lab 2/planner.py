from math import exp, sqrt
# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0, 0.0]):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)
        
        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner()


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        theta = goalPoint[2]
        return x, y, theta

    # Part 6: Implement the trajectories here
    def trajectory_planner(self):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]

        # cover the sigma = 1 / (1 + e^-x)
        trajectory = [[0.1 * i , 1 / (1 + exp(-0.1 * i))] for i in range(10)]

        # cover the y = x^2, note we scaled it during the lab section to 0.5 after asking TAs in lab to make sure it fits in space 
        trajectory = [[0.1 * i, pow(0.1 * i, 2)] for i in range(10)]

        return trajectory
