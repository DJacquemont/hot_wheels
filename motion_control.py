'''
MOTION CONTROL v2.3

-------- HOW TO USE ------
First declare a Motion Control instance with optimal trajectory to follow and actual state of the robot
    mc = MotionControl(opt_traj, x_pos, y_pos, angle, err_dist=1, err_angle=np.pi/100):

Then update the robot's wheel speed (left and right) by calling one of the following
    mc.update_global(self, x_pos, y_pos, ori)
    mc.update_local(self, sensors)

These functions return nothing, but update the attributes of the robot:
    thymio_l_speed = mc.l_speed
    thymio_r_speed = mc.r_speed
----------  END ----------

------ HOW IT WORKS ------
The optimal trajectory is a list of Node instances. A Node has an id, a x and a y position.
The opt_traj looks the following :

    [Node(0,10,10), Node(1, 25, 20), Node(2, 25, 30)]
           |               |                |
      First node      Second Node       Goal Node

The motion control instance has a robot position attribute as the following:
    robot_pos = Node(id, x_pos, y_pos)
    robot_ori = angle
with.
    id : the actual node of the opt_traj
    x,y : positions
    angle : orientation of the robot

When calling update functions we compute the speed of the wheels regarding
the distance of the robot to the Node with the next id in the opt_traj. If the robot is
within a small circle around the next Node, the id of the robot_pos increments by 1 to
track the next Node in opt_traj.
----------  END ----------

inputs:
    opt_traj - a list of optimal points to follow
    current_pos_x - the current x position of the robot
    current_pos_y - the current y position of the robot
    current_angle - the current angle of the robot
    prox - the proximity sensors (used for local avoidance behavior)
        
outputs:
    l_speed : speed of the left motor
    r_speed : speed of the right motor
'''

import numpy as np
import vision
import AlgoGlobNav as gn

################ constants ################
ROBOT_WIDTH = 9
SPEED_MAX_ASEBA = 500 # [aseba unit]
SPEED_MAX_CMS = 20    # [cm/s]
SPEED_RATIO = SPEED_MAX_ASEBA/SPEED_MAX_CMS # [aseba unit*s/cm]
###########################################


class Node:
    def __init__(self,i,x,y):
        '''
        A Node is a point on the map, it has:
            id - the id of the node
            x - the x position on the map [cm]
            y - the y position on the map [cm]
            norm2 - its nominal distance to the origin [cm]
            angle - its nominal angle in rad [rad]
        '''
        self.id = i
        self.x = x
        self.y = y
        self.norm2 = np.sqrt(x**2 + y**2)
        self.angle = np.arctan2(y,x)
        
    def dist(self, p2):
        '''
        Computes the distance from another node [cm]
        '''
        return np.sqrt((self.x-p2.x)**2 + (self.y-p2.y)**2)
    
    def diff_angle(self, p2):
        '''
        Computes the difference in angles with another node [rad]
        '''
        return self.angle - p2.angle
    
    def join_angle(self, p2):
        '''
        Compute the angle of the vector joining node1 and node1
        '''
        join_node = Node(i=0, x=p2.x-self.x, y=p2.y-self.y)
        return join_node.angle
        
class Trajectory:
    def __init__(self, points):
        '''
        A Trajectory is a sequence of nodes, it has:
            points - numpy array of Node instances
            total_len - total distance of the trajectory
        '''
        self.points = np.array(points)
        self.total_len = np.sum([points[idx].dist(points[idx+1]) for idx in range(len(points)-1)])
                
class MotionControl:
    def __init__(self, traj, x_pos, y_pos, angle, nodes, nodeCon, maskObsDilated, goal, err_dist=2, err_angle=np.pi/8
    ):
        '''
        The MotionControl class allows to compute the speed of the wheel motors, it has:
            opt_traj - the optimal trajectory to follow
            robot_pos -
                id - the index of the actual position to track what is the next node to reach
                x - actual x position of the robot [cm]
                y - actual y position of the robot [cm]
                norm2 - its nominal distance to the origin [cm]
                angle - vector angle of the robot position [rad]
            robot_ori - the current orientation of the robot [rad]
            err_dist - the allowed error done when targeting a point [cm]
            err_angle - the allowed error done while pointing at a target [cm]
            state - the current state of the FSM for switching between global and local behavior
            l_speed - computed speed of the left wheel [aseba unit]
            r_speed - computed speed of the right wheel [aseba unit]

            goal, optiam_path, nodes, nodeCon, maskIbsDilated - variables used for Kalman and Optimal path computation 
        '''
        self.opt_traj = Trajectory([])
        i=0
        for pos in traj:
            self.opt_traj.points = np.append(self.opt_traj.points, Node(i,pos[0],pos[1]))
            i+=1
        self.robot_pos = Node(0, x_pos, y_pos)
        self.robot_ori = angle
        self.err_dist = err_dist
        self.err_angle = err_angle
        self.state = 'global'
        self.l_speed = 0
        self.r_speed = 0

        self.goal = goal
        self.optimal_path = traj
        self.nodes = nodes
        self.nodeCon = nodeCon
        self.maskObsDilated = maskObsDilated

    def move_fwd(self, target):
        ''' 
        Move foward function: gives instructions for the robot to move from a point A to B (using a proportional controller)
        Inputs:
            self.current_pos - coordinates of the current robot position [cm]
            target - coordinates of the goal position [cm]
            self.current_angle - current oriantation of the robot [rad]
        Outputs:
            self.l_speed - speed for left motor [aseba unit]
            self.r_speed - speed for right motor [aseba unit]
        '''
        dist_off = self.robot_pos.dist(target) # difference between desired position and current one

        des_angle = self.robot_pos.join_angle(target)   # desired angle
        angle_off = des_angle - self.robot_ori       # difference between desired angle and current orientation of the robot
        
        # TUNE PARAMETERS FOR PROPORTIONAL CONTROLLER
        K_dist = 0.04                #further, faster
        K_ori = 2                    #less oriented, higher motor difference
        speed_offset = 5 # [cm/s]    #constant speed offset

        self.l_speed = SPEED_RATIO * (speed_offset + K_dist*dist_off - K_ori*angle_off) #go faster if angle is negative
        self.r_speed = SPEED_RATIO * (speed_offset + K_dist*dist_off + K_ori*angle_off) #go faster if angle is positive

    def pivot(self, target):
        ''' 
        Pivot function: gives instructions for the robot to pivot using 3 points
        Inputs:
            current_point - coordinates of the current robot position [cm]
            target - coordinates of the next position [cm]
            current_angle - current angle of the robot [rad]
        Outputs:
            l_speed - speed for left motor [aseba unit]
            r_speed - speed for right motor [aseba unit]

        '''
        #desired angle
        des_angle = self.robot_pos.join_angle(target)

        #angle between robot and target point: angle_off [-pi, +pi]
        angle_off = (des_angle - self.robot_ori)
        angle_off = angle_off if (angle_off > -np.pi) and (angle_off < np.pi) else \
                    angle_off - 2 * np.pi if (angle_off > np.pi) else \
                    angle_off + 2 * np.pi

        # TUNE PARAMETERS FOR PROPORTIONAL CONTROLLER
        K_piv = 0.8                   #less oriented, faster
        speed_offset = 0.8 # [cm/s]   #constant speed offset

        self.l_speed = -(np.sign(angle_off)*speed_offset + K_piv*(angle_off)) * SPEED_RATIO
        self.r_speed = +(np.sign(angle_off)*speed_offset + K_piv*(angle_off)) * SPEED_RATIO

    def update_global(self):
        '''
        Updates the speed of the the robot wheels using the position of the robots in case we are in a GLOBAL path search
        Inputs:
            x_pos, y_pos - estimated position of the robots from the kalman filter [cm]
            ori - estimated orientation of the robot [rad]
        Outputs:
            l_speed - speed for the left motor [aseba unit]
            r_speed - speed for the right motor [aseba unit]
        '''
        # if the goal point is reached --> stop the wheels 
        if self.robot_pos.dist(self.opt_traj.points[len(self.opt_traj.points)-1]) < self.err_dist: 
            self.l_speed, self.r_speed = 0, 0

        # if the goal point isn't reached yet 
        else:
            next_point = self.opt_traj.points[self.robot_pos.id+1]          # next point to follow
            if self.robot_pos.dist(next_point) < self.err_dist:             # if next point is reached
                self.robot_pos.id += 1                                      # update the id of the robot position
                next_point = self.opt_traj.points[self.robot_pos.id+1]      # update the next point to follow

            # if the angle between the robot orientation and the next point is too high
            if abs(self.robot_ori - self.robot_pos.join_angle(next_point)) > self.err_angle:
                self.pivot(next_point)      # then pivot to face the next point
            else:
                self.move_fwd(next_point)   # else go forward the next point

    def update_local_pivot(self):
        ''' 
        Local pivot function: gives instructions for the robot to pivot left until no more sensor is detected
        Inputs:
            None
        Outputs:
            l_speed - speed for left motor [aseba unit]
            r_speed - speed for right motor [aseba unit]

        '''
        self.l_speed = -30
        self.r_speed = +30

    def update_local_fwd(self):
        ''' 
        Local pivot function: gives instructions for the robot to go forward 15cm to pass the obstacle
        Inputs:
            None
        Outputs:
            l_speed - speed for left motor [aseba unit]
            r_speed - speed for right motor [aseba unit]

        '''
        self.l_speed = +60
        self.r_speed = +60

    def update_motion(self, x_pos, y_pos, ori, prox, vid):
        '''
        FSM switching from local to global behavior
        Inputs:
            x_pos, y_pos - estimated position of the robots from the kalman filter [cm]
            ori - estimated orientation of the robot [rad]
            sensors - front sensors on the thymio [aseba unit]
            opt_path - optimal path to follow in case it is actuated
        Outputs:
            calls global/local update function:
                l_speed - speed for the left motor [aseba unit]
                r_speed - speed for the right motor [aseba unit]
        '''
        self.robot_pos.x = x_pos    # first of all, update the state of the robot
        self.robot_pos.y = y_pos
        self.robot_ori = ori
        prox = prox[0:5]
        
        # GLOBAL STATE
        if self.state == 'global':
            if any(prox):                                       #if the sensors detect something
                self.state = 'local_pivot'                      #switch to local pivot state
            else:                                               #else
                self.update_global()                            #follows global path

        #LOCAL PIVOT STATE
        if self.state == 'local_pivot':                         
            if any(prox):                                       #if sensors detect something
                self.update_local_pivot()                       #continue pivoting
            else:                                               #if sensors detect nothing
                self.init_local_pos = Node(0,x_pos,y_pos)       #save the position of the robot
                self.state = 'local_fwd'                        #go foward to pass the obstacle

        #LOCAL FORWARD STATE            
        if self.state == 'local_fwd':       
            if self.init_local_pos.dist(self.robot_pos) < 15:   #if Thymio hasn't travelled 15cm yet
                if not(any(prox)):                              #and the sensors don't detect anything
                    self.update_local_fwd()                     #go forward
                else:                                           #if the sensors detect something
                    self.state = 'local_pivot'                  #switch to local pivot state
            else:                                               #if Thymio has gone 15cm forward
                try :                                           #try recomputing the optimal path
                    self.nodes, self.nodeCon, self.maskObsDilated, optimal_path = gn.opt_path(vid, self.goal)
                    self.optimal_path = optimal_path*100
                    self.opt_traj = Trajectory([])
                    i=0
                    for pos in self.optimal_path:
                        self.opt_traj.points = np.append(self.opt_traj.points, Node(i,pos[0],pos[1]))
                        i+=1
                    self.robot_pos.id = 0
                except:                                         #if can't recompute, just skip and keep old optimal path
                    print("Skipped optimal path update")
                self.state = 'global'                           #and switch to global navigation
