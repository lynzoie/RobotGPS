import numpy as np
import os
import time 

from ViveServer.Client import Client
from Motion import Motion

a_arr = []
b_arr = []

class GPS(object):
    def __init__(self):
        self.x = 0          # x center of robot in m
        self.y = 0          # y center of robot in m
        self.theta = 0      # angle from x-axis that robot is facing in degrees, -180 < self.theta < +180
        
        self.v = 0          # linear velocity, optional attribute
        self.w = 0          # angular velocity, opitional attribute

        self.Cx = 0         # x center between the two wheels in m, optional attribute
        self.Cy = 0         # y center between the two wheels in m, optional attribute

    #### Write gps coordinates to file ####
    def output_gps_coords(self):
        out_arr = np.vstack((a_arr,b_arr))
        with open('test_navrobot.txt','w') as f:
            for item in out_arr:
                f.write("%s\n" % item)

    #### Calculate angle that robot is facing ####
    def calc_theta(self, Ax, Ay, Bx, By):
        phi = np.degrees(np.arctan2(By-Ay,Bx-Ax))
        temp_theta = phi + 90
        theta = np.where(temp_theta>180, temp_theta-360, temp_theta)
        return theta

    #### Calculate robot's center x,y and theta ####
    def tracker_to_xytheta(self, x1, z1, x2, z2):
        Ax = x1
        Ay = z1
        Bx = x2
        By = z2

        self.theta = self.calc_theta(Ax, Ay, Bx, By)
        self.Cx = ((Bx - Ax) / 2) + Ax
        self.Cy = ((By - Ay) / 2) + Ay

        r = 0.05        # assuming the center of the robot is exactly 50mm from the wheel axis
        # offset to circle at center Cx,Cy
        self.x = self.Cx - (r*np.cos(np.radians(self.theta)))
        self.y = self.Cy - (r*np.sin(np.radians(self.theta)))

    #### Update GPS object's x, y, and theta ####
    def get_robot_pose(self, c):
        gps = c.get_trackers_coordinates()
        a = list(gps['tracker_1'])
        b = list(gps['tracker_2'])
        a_arr.append(a[0:3])
        b_arr.append(b[0:3])
        self.tracker_to_xytheta(a[0],a[2],b[0],b[2])   # grab current pose of robot

    #### Calculate correct turn angle to face desired phi ####
    def calc_turn_angle(self, phi, robot, move_robot):
        # Rotate to face destination
        conv_theta = np.where(self.theta >= 0, self.theta, 360+self.theta)       # convert negative angles to range 0<theta<360
        conv_phi = phi if phi >= 0 else 360+phi                                  # convert negative angles to range 0<phi<360
        turn_angle = conv_theta - conv_phi

        # turn angle<0 turns CCW and vice versa
        if (turn_angle > 180):
            turn_angle = 360 - turn_angle
        elif (turn_angle < -180):
            turn_angle = -(360 + turn_angle)
        else:
            turn_angle = -turn_angle

        move_robot.turn(a=turn_angle, bot=robot, smooth_stop=False)     # pass angle as degrees

    #### Rotate to correct angle ####
    def turn_to_angle(self, c, new_ang, robot, move_robot):
        while (abs(self.theta-new_ang)>1):                              # feedback system to fix angle
            self.calc_turn_angle(new_ang, robot, move_robot)    
            self.get_robot_pose(c)

    #### Calculate dest_angle robot needs to face to travel straight to destination ####
    def dest_angle(self, conv_x, conv_y, new_x, new_y):                 
        dest_ang = np.degrees(np.arctan2(new_y-conv_y, new_x-conv_x))
        x = dest_ang + 180 if dest_ang > 0 else (360+dest_ang) + 180
        if x > 180:
            dest_ang = x - 360
        return dest_ang

    #### Move straight to destination ####
    def move_to_dest(self, d, v=100, w=0, robot=None, c=None):          
        self.v = v  # in mm/sec
        self.w = w  # in ang/sec
        t = d/v     # time it takes to move d mm with v mm/sec
        
        wheelbase = 235
        vL = v - (wheelbase/2)*np.radians(w)
        vR = v + (wheelbase/2)*np.radians(w)

        start = time.monotonic()        
        while (time.monotonic() < start + t):
            robot.drive_direct(int(np.round(vL)) ,int(np.round(vR)))
            self.get_robot_pose(c)

        robot.drive_direct(0,0)

    #### Main function to use to travel robot to a specific coordinate ####
    def travel(self, robot, c, new_x:'x-coordinate in m', new_y:'y-coordinate in m', new_ang:'final angle (in degrees), range: -180<x<+180'):
        # Convert desired xy from m to mm
        new_x = new_x * 1000
        new_y = new_y * 1000
        p2 = np.array((new_x, new_y))

        move_robot = Motion.RobotTranslator(robot)

        #### STEPS TO GET TO DESTINATION ####
        # 1. First, have robot face destination
        # 2. Second, have the robot start moving forward to destination (move towards minimum distance between the two points)
        # 3. Check if we're at destination, else restart steps 1 and 2
        # 4. Once at destination (or close enough within threshold), rotate robot to final angle
        
        print("Moving to target location: (" , new_x , "," , new_y , ") with final angle: ", new_ang)
        
        # 3. repeat steps 1 and 2 if needed
        threshold = 15      # difference between desired coordinate and measured coordinate that is acceptable
        while (abs(new_x - (self.x*1000)) >= threshold or abs(new_y - (self.y*1000)) >= threshold):
            # Get current robot's coordinates in mm
            self.get_robot_pose(c)
            conv_x = self.x * 1000
            conv_y = self.y * 1000       
            p1 = np.array((conv_x, conv_y))

            # 1. rotate to destination
            print("Rotating to face destination... \n")
            dest_ang = self.dest_angle(conv_x, conv_y, new_x, new_y)
            self.turn_to_angle(c, dest_ang, robot, move_robot)

            # 2. move to destination
            print("Moving towards destination... \n")
            dist = np.linalg.norm(p2-p1)
            self.move_to_dest(d=dist, robot=robot, c=c)

            print("X difference: ", abs(new_x-(self.x*1000)), " Y difference: ", abs(new_y-(self.y*1000)))
            print("Distance traveled: ", dist, "\n")

        # 4. rotate to final angle at destination
        print("Rotate at destination...")
        self.turn_to_angle(c, new_ang, robot, move_robot)

        # write gps data to text file
        self.output_gps_coords()
        
