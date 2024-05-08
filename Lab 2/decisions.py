# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once, shutdown
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):

    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):
        super().__init__("decision_maker")

        #Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher= self.create_publisher(Twist, "/cmd_vel", 10)

        publishing_period=1/rate
        
        # Instantiate the controller
        # Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            # Note these parameters are some parameters we were testing for simulations
            self.controller=controller(klp=1.1, klv=0.005, kli=1.0, kap=1.1, kav=0.005, kai=1.0)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=1.1, klv=0.005,kli=1.0, kap=1.1, kav=0.005, kai=1.0)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rwwSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.
        spin_once(self.localizer)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # Part 3: Check if you reached the goal
        reached_goal = False
        pose = self.localizer.getPose()

        # if we are doing trajectory then the self.
        if type(self.goal[0]) == list:
            if abs(pose[0] - self.goal[-1][0]) < 0.025 and abs(pose[1] - self.goal[-1][1]) < 0.025: 
                reached_goal= True
                self.publisher.publish(Twist())

        # set the convergence criteria for having reached
        elif abs(pose[0] - self.goal[0]) < 0.025 and abs(pose[1] - self.goal[1]) < 0.025 and abs(pose[2] - self.goal[2]) < 0.025: 
            reached_goal= True
            self.publisher.publish(Twist())

        if reached_goal:
            self.publisher.publish(vel_msg)
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #Part 3: exit the spin
            raise SystemExit("Exit the spin")

        # get the values to go the goal
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        #Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)      

import argparse


def main(args=None):
    
    init()

    # Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)
    

    # Part 3: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [0.0, 0.0, 0.0], rate=10,motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [-0.5, -0.5, -0.5], rate=10,motion_type=TRAJECTORY_PLANNER)
    # using this one for extra testing
    elif args.motion.lower() == "point2":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, [-0.5, -0.5, -0.5], rate=10,motion_type=POINT_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")
        try:
            DM.destroy_node()
        except Exception as e:
            print("Couldn't destroy the node: {}".format(e))
    shutdown()


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
