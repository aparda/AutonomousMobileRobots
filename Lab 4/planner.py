
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, mapName="room"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        # NOTE: i put in laser_sig based on the initial vals in map manip, might nieed to change this
        self.m_utilites=mapManipulator(laser_sig=0.3)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        originPose = self.m_utilites.position_2_cell(self.m_utilites.getOrigin())
        #print("THE POSES: ")
        #print(startPose)
        #print(endPose)
        # print(type(startPose))
        # startPose = list(startPose)
        # endPose = list(endPose)
        # TODO PART 5 convert the cell pixels into the cartesian coordinates
        # Not sure about this part?
        # Path = list(map(self.m_utilites.cell_2_position, self.costMap))

        startNode = Node(parent=None, position=startPose)
        endNode = Node(parent=None, position=endPose)
        #cell_path = search(self.costMap, startNode, endNode, originPose)
        #Path = list(map(self.m_utilites.cell_2_position, cell_path))
        path=search(self.costMap, startNode, endNode, 0, MANHAT)
        Path = list(map(self.m_utilites.cell_2_position, path))
        # PART 5 return the path as list of [x,y]
        return Path


if __name__=="__main__":

    m_utilites=mapManipulator()
    
    map_likelihood=m_utilites.make_likelihood_field()
  
    
    '''
    path=search(map_likelihood, 0, [70,90], [100,90])
    
    print( list(map(m_utilites.cell_2_position, path)))
    '''
