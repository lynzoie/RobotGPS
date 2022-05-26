import numpy as np
import os

from ViveServer.Client import Client
from Motion import Motion

IP_ADDRESS = '192.168.8.166' #IP address of elisive-calf on Alfred CAPCOM

class GPS(object):
    def __init__(self):
        self.x = 0          # x center of robot
        self.y = 0          # y center of robot
        self.theta = 0      # angle of robot
        
        self.v = 0          # linear velocity, optional attribute
        self.w = 0          # angular velocity, opitional attribute

        self.Cx = 0         # x center between the two wheels, optional attribute
        self.Cy = 0         # y center between the two wheels, optional attribute

    def calc_theta(self, Ax, Ay, Bx, By):
        phi = np.degrees(np.arctan2(By-Ay,Bx-Ax))
        temp_theta = phi + 90
        new_theta = np.where(temp_theta>180, temp_theta-360, temp_theta)
        theta = np.radians(new_theta)
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
    
    def travel(self, robot, new_x:'x-coordinate in m', new_y:'y-coordinate in m'):
        # Convert desired xy from m to mm
        new_x = new_x * 1000
        new_y = new_y * 1000

        # Calculate distance and angle robot should turn to get to target
        p1 = np.array((self.x,self.y))
        p2 = np.array((new_x,new_y))
        dist = np.linalg.norm(p1 - p2)
        
        # Rotate to face destination
        phi = np.degrees(np.arctan2(p2[1]-p1[1],p2[0]-p1[0]))
        turn_angle = phi
        move_robot = Motion.RobotTranslator(robot)
        move_robot.turn(a=turn_angle,bot=robot)     # pass angle as degrees

        # Start moving to destination
        move_robot.move(d=dist/2,bot=robot)
        
