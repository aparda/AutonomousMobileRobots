import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from typing import List

MANHAT = 0
EUCLID = 1

class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position

#This function return the path of the search
def return_path(current_node,maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    print(f"Print path{path}")
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path


def search(maze, start, end, mazeOrigin, hType=EUCLID):

    print("searching ....")
    print(f"Maze origin {mazeOrigin}")

    maze = maze.T
    
    
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """

    # PART 4 Create start and end node with initized values for g, h and f
    print("THE INTIAL POSITIONS")
    start_node = start
    start_node.g = 0
    start_node.h = 0
    if EUCLID:
        start_node.h = np.linalg.norm(np.array(end.position)-np.array(start.position))

    if MANHAT:
        start_node.h = abs(end.position[0] - start.position[0]) + abs(end.position[1] - start.position[1])

    start_node.f = start_node.g + start_node.h
    print(start_node.position)

    
    end_node = end
    end_node.g = 0
    end_node.h = 0
    end_node.f = 0
    print(end_node.position)

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    yet_to_visit_list = []  
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = []
    
    # Add the start node
    yet_to_visit_list.append(start_node)
    
    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    
    # PART 4 what squares do we search . serarch movement is left-right-top-bottom 
    #(4 movements) from every positon
    move  =  [[0, 1], # go up
              [-1, 0], # go left
              [0, -1], # go down
              [1, 0], # go right
              [-1, 1], # go up left
              [-1, -1], # go down left
              [1, 1], # go up right
              [1, -1]] # go down right


    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    # PART 4 find maze has got how many rows and columns 
    no_rows, no_columns = np.shape(maze)
    

    # Loop until you find the end
    
    while len(yet_to_visit_list) > 0:
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        # print(len(yet_to_visit_list), len(visited_list))
        outer_iterations += 1    

        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:
            return return_path(current_node,maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move: 

            # PART 4 Get node position by updating it with the 8 possible choices
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # PART 4 Make sure within range (check if within maze boundary)
            # if node_position[0] < no_rows and node_position[1] < no_columns and node_position[0] >= mazeOrigin[0] \
            #    and node_position[1] >= mazeOrigin[1]:
            #     pass
            # else:
            #     continue

            if node_position[0] < no_rows and node_position[1] < no_columns and node_position[0] >= 0 \
               and node_position[1] >= 0:
                pass
            else:
                print("we are going out of bounds")
                continue

            # Make sure walkable terrain
            if maze[node_position[0],node_position[1]] > 0.6:
                print("this is not walkable")
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        # print("Child length ", len(children))
        for child in children:

            # print(f"Visited list in children loop\n{visited_list}")
  
            # PART 4 Child is on the visited list (search entire visited list)
            if len(visited_list) > 0:
                # should be faster if a set for lookup
                if child in visited_list:
                    continue

            # print(f"Children list past if")
            # PART 4 Create the f, g, and h values
            # g is the cost on the edge of the graph to go to the node
            # print(f'child g {child.parent.g}')
            if hType == EUCLID:
                child.g = child.parent.g + np.linalg.norm(np.array(child.parent.position) - np.array(child.position)) # maze[node_position[0], node_position[1]]
                child.h = sqrt(pow(child.position[0] - end_node.position[0], 2) \
                            + pow(child.position[1] - end_node.position[1], 2))
            ## Heuristic costs calculated here
            # manhattan
            if hType == MANHAT:
                child.g = child.parent.g + np.abs(child.parent.position[0] - child.position[0]) + np.abs(child.parent.position[1] - child.position[1]) 
                child.h = abs(child.position[0] - end_node.position[0]) \
                + abs(child.position[1] - end_node.position[1])

            # the total cost
            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            if len([i for i in yet_to_visit_list if child == i and child.g >= i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)
            #print("How many times do we actaully get here")
