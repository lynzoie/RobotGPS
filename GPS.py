import numpy as np
import os

from ViveServer.Client import Client
from Motion import Motion

a_arr = []
b_arr = []

class GPS(object):
    def __init__(self):
        self.x = 0          # x center of robot
        self.y = 0          # y center of robot
        self.theta = 0      # angle of robot in degrees
        
        self.v = 0          # linear velocity, optional attribute
        self.w = 0          # angular velocity, opitional attribute

        self.Cx = 0         # x center between the two wheels, optional attribute
        self.Cy = 0         # y center between the two wheels, optional attribute


    def calc_theta(self, Ax, Ay, Bx, By):
        phi = np.degrees(np.arctan2(By-Ay,Bx-Ax))
        temp_theta = phi + 90
        theta = np.where(temp_theta>180, temp_theta-360, temp_theta)
        return theta


    def find_centerpoint(self, Ax, Ay, Bx, By):
        Cx = ((Bx - Ax) / 2) + Ax
        Cy = ((By - Ay) / 2) + Ay
        return Cx, Cy


    def tracker_to_xytheta(self, x1, z1, x2, z2):
        # x-center of robot is in the middle of the wheel axis (on x-axis)
        # y-center of robot is at 50mm = 0.05m above wheel axis
        # Theta is angle from x-axis that the robot is facing
        Ax = x1
        Ay = z1

        Bx = x2
        By = z2

        self.Cx, self.Cy = self.find_centerpoint(Ax, Ay, Bx, By)
        self.theta = self.calc_theta(Ax, Ay, Bx, By)

        r = 0.05        # assuming the center of the robot is exactly 50mm from the wheel axis
        # offset to circle at center Cx,Cy
        self.x = self.Cx - (r*np.cos(self.theta))        
        self.y = self.Cy - (r*np.sin(self.theta))


    def get_robot_pose(self,c):
        gps = c.get_trackers_coordinates()
        a = list(gps['tracker_1'])
        b = list(gps['tracker_2'])
        a_arr.append(a[0:3])
        b_arr.append(b[0:3])
        
        self.tracker_to_xytheta(a[0],a[2],b[0],b[2])   # grab current pose of robot
    

    def output_gps_coords(self):
        out_arr = np.vstack((a_arr,b_arr))
        with open('test_navrobot.txt','w') as f:
            for item in out_arr:
                f.write("%s\n" % item)


    def rot_to_dest(self,cur_x,cur_y,new_x,new_y, robot, move_robot):
        # Rotate to face destination
        phi = np.degrees(np.arctan2(new_y-cur_y , new_x-cur_x))
        
        alpha = phi - self.theta               # TODO: fix alpha so we face destination. Consider logic of alpha<0 turns CCW and vice versa
        # move_robot.turn(a=alpha,bot=robot)     # pass angle as degrees


    def travel(self, robot, c, new_x:'x-coordinate in m', new_y:'y-coordinate in m'):
        # Get inital coordinates in mm
        self.get_robot_pose(c)
        conv_x = self.x * 1000
        conv_y = self.y * 1000
        print('Conv self.x and self.y: ', conv_x,conv_y)
        
        # Convert desired xy from m to mm
        new_x = new_x * 1000
        new_y = new_y * 1000
        print('Desired x and y: ', new_x,new_y)

        move_robot = Motion.RobotTranslator(robot)
        
        # rotate to face destination
        self.rot_to_dest(conv_x,conv_y,new_x,new_y,robot,move_robot)     
        self.get_robot_pose(c)


        
