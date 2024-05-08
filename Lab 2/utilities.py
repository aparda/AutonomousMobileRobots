from math import atan2, sqrt

M_PI=3.1415926535

class Logger:
    
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        
        self.filename = filename

        with open(self.filename, 'w') as file:
            
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            
            vals_str=""
            
            for value in values_list:
                vals_str+=f"{value}, "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table
    
    

# Part 3: Implement the conversion from Quaternion to Euler Angles
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    quat = [x, y, z, w]
    """
    siny_cosp = 2 * (quat.w*quat.z + quat.x*quat.y)
    cosy_cosp = 1 - 2*(quat.y*quat.y + quat.z*quat.z)
    yaw = atan2(siny_cosp, cosy_cosp)

    # just unpack yaw
    return yaw


#Part 4: Implement the calculation of the linear error
def calculate_linear_error(current_pose, goal_pose):
        
    # Compute the linear error in x and y
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y,theta]
    # Remember to use the Euclidean distance to calculate the error.
    error_linear= sqrt(pow((current_pose[0]- goal_pose[0]),2)+ pow((current_pose[1] - goal_pose[1]),2))

    return error_linear

#Part 4: Implement the calculation of the angular error
def calculate_angular_error(current_pose, goal_pose, is_trajectory=False):

    # Compute the angular error
    # Remember that current_pose = [x,y, theta, time stamp] and goal_pose = [x,y,theta]
    # Remember that this function returns the difference in orientation between where the robot currently faces and where it should face to reach the goal

    linear_error = calculate_linear_error(current_pose, goal_pose)
    error_angular = 0

    # two cases, we haven't gotten to the spot and need to adjust the heading to reach
    if linear_error > 0.025:
        '''
        first we need to make sure we're pointing towards the right direction (imagine both are at heading 0 but the x and y are offset)
        we would end up driving forwards and backwards and never reach our goal.
        '''
        opp = goal_pose[1] - current_pose[1]
        adj = goal_pose[0] - current_pose[0]
        desired_theta = atan2(opp,adj)
        '''
        next we need to align this theta with our theta. This is because we drive using polar so we need to handle theta first and then drive
        to the point
        '''

        error_angular = desired_theta - current_pose[2]

    # or we have gotten to the point, now we just have to adjust our heading
    else:
        error_angular = goal_pose[2] - current_pose[2]

    # Remember to handle the cases where the angular error might exceed the range [-π, π]  
    if error_angular <= -M_PI:
        error_angular += 2*M_PI
    elif error_angular >= M_PI:
        error_angular -= 2*M_PI

    return error_angular
