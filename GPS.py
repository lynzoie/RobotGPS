import numpy as np
import os

from ViveServer.Client import Client
from Motion import Motion

a_arr = []
b_arr = []

class GPS(object):
    def __init__(self):
        self.x = 0          # x center of robot in m
        self.y = 0          # y center of robot in m
        self.theta = 0      # angle of robot in degrees, -180 < self.theta < +180
        
        self.v = 0          # linear velocity, optional attribute
        self.w = 0          # angular velocity, opitional attribute

        self.Cx = 0         # x center between the two wheels in m, optional attribute
        self.Cy = 0         # y center between the two wheels in m, optional attribute


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
        self.x = self.Cx - (r*np.cos(np.radians(self.theta)))
        self.y = self.Cy - (r*np.sin(np.radians(self.theta)))


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

    def rot_to_angle(self, phi, robot, move_robot):
        # Rotate to face destination
        conv_theta = np.where(self.theta < 0, 360+self.theta, self.theta)       # convert negative angles to range 0<theta<360
        conv_phi = phi if phi > 0 else 360+phi                                  # convert negative angles to range 0<phi<360

        turn_angle = conv_theta - conv_phi

        # turn angle<0 turns CCW and vice versa
        if (turn_angle > 180):
            turn_angle -= 360
        elif (turn_angle < -180):
            turn_angle += 360
        else:
            turn_angle = -turn_angle

        print("Turn angle: ", turn_angle)
        move_robot.turn(a=turn_angle,bot=robot,smooth_stop=False)     # pass angle as degrees


    def travel(self, robot, c, new_x:'x-coordinate in m', new_y:'y-coordinate in m', new_ang:'final angle (in degrees), range: -180<x<+180'):
        # Get inital coordinates in mm
        self.get_robot_pose(c)
        conv_x = self.x * 1000
        conv_y = self.y * 1000
        print('Conv self.x and self.y: ', conv_x,conv_y)
        print("Current angle: ", self.theta)        
        
        # Convert desired xy from m to mm
        new_x = new_x * 1000
        new_y = new_y * 1000
        print(' ')
        print('Desired x and y: ', new_x,new_y)
        print("Desired new angle: ", new_ang)
        print(' ')
        move_robot = Motion.RobotTranslator(robot)

        # rotate to final pose angle
        while (abs(self.theta-new_ang)>1):     # feedback system to fix angle
            self.rot_to_angle(new_ang, robot, move_robot)    
            self.get_robot_pose(c)
            print("Actual theta: ", self.theta) 

        # write gps data to text file
        self.output_gps_coords()


        
