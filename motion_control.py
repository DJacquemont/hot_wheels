'''
MOTION CONTROL v1.1
 -- commented classes

inputs:
    opt_traj - a list of optimal points to follow
    current_pos_x - the current x position of the robot
    current_pos_y - the current y position of the robot
    current_angle - the current angle of the robot
        
outputs:
    l_speed : speed of the left motor
    r_speed : speed of the right motor
'''

import numpy as np
from matplotlib import pyplot as plt

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
            points - numpy array of nodes
            total_len - totol distance of the trajectory
        '''
        self.points = np.array(points)
        self.total_len = np.sum([points[idx].dist(points[idx+1]) for idx in range(len(points)-1)])
                
class MotionControl:
    def __init__(self, opt_traj, x_pos, y_pos, angle, err_dist=5, err_angle=np.pi/8):
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
            l_speed - computed speed of the left wheel [aseba unit]
            r_speed - computed speed of the right wheel [aseba unit]
        '''
        self.opt_traj = opt_traj
        self.robot_pos = Node(0, x_pos, y_pos)
        self.robot_ori = angle
        self.err_dist = err_dist
        self.err_angle = err_angle
        self.l_speed = 0
        self.r_speed = 0
        
    def move_fwd(self, goal_node):
        ''' 
        Move foward function: gives instructions for the robot to move from a point A to B (using a proportional controller)
        Inputs:
            self.current_pos - coordinates of the current robot position [cm]
            goal_node - coordinates of the goal position [cm]
            self.current_angle - current oriantation of the robot [rad]
        Outputs:
            self.l_speed - speed for left motor [aseba unit]
            self.r_speed - speed for right motor [aseba unit]
        '''
        dist_off = self.robot_pos.dist(goal_node) # difference between desired position and current one

        des_angle = self.robot_pos.join_angle(goal_node)   # desired angle
        angle_off = des_angle - self.robot_ori       # difference between desired angle and current orientation of the robot
        
        # TUNE PARAMETERS
        K_dist = 0.2
        K_ori = 5
        speed_offset = 1 # [cm/s]

        self.l_speed = SPEED_RATIO * (speed_offset + K_dist*dist_off - K_ori*angle_off) #go faster if angle is negative
        self.r_speed = SPEED_RATIO * (speed_offset + K_dist*dist_off + K_ori*angle_off) #go faster if angle is positive

    def pivot(self, next_point):
        ''' 
        Pivot function: gives instructions for the robot to pivot using 3 points
        Inputs:
            current_point - coordinates of the current robot position [cm]
            next_point - coordinates of the next position [cm]
            current_angle - current angle of the robot [rad]
        Outputs:
            l_speed - speed for left motor [aseba unit]
            r_speed - speed for right motor [aseba unit]

        '''
        des_angle = self.robot_pos.join_angle(next_point)
        angle_off = des_angle - self.robot_ori
        
        # TUNE PARAMETERS
        K_piv = 1
        speed_offset = 1 # [cm/s]

        self.l_speed = -(np.sign(angle_off)*speed_offset + K_piv*(angle_off)) * SPEED_RATIO
        self.r_speed = +(np.sign(angle_off)*speed_offset + K_piv*(angle_off)) * SPEED_RATIO

    def update_global(self, x_pos, y_pos, ori):
        '''
        Updates the speed of the the robot wheels using the position of the robots in case we are in a GLOBAL path search
        Inputs:
            x_pos, y_pos - estimated position of the robots from the kalman filter [cm]
            ori - estimated orientation of the robot [rad]
        Outputs:
            l_speed - speed for the left motor [aseba unit]
            r_speed - speed for the right motor [aseba unit]
        '''
        self.robot_pos.x = x_pos
        self.robot_pos.y = y_pos
        self.robot_ori = ori
        
        if self.robot_pos.dist(self.opt_traj.points[len(self.opt_traj.points)-1]) < self.err_dist: #if goal point reached -> end
            self.l_speed, self.r_speed = 0, 0
        else:
            next_point = self.opt_traj.points[self.robot_pos.id+1]
            if self.robot_pos.dist(next_point) < self.err_dist:
                self.robot_pos.id += 1
                next_point = self.opt_traj.points[self.robot_pos.id+1]

            if abs(self.robot_ori - self.robot_pos.join_angle(next_point)) > self.err_angle:
                self.pivot(next_point)
            else:
                self.move_fwd(next_point)
                
    def update_local(self, sensors):
        '''
        Updates the speed of the the robot wheels using the position of the robots in case we are in a LOCAL path search
        Inputs:
            sensors - front sensors on the thymio [aseba unit]
        Outputs:
            l_speed - speed for the left motor [aseba unit]
            r_speed - speed for the right motor [aseba unit]
        '''
        pass