from hashlib import new
import numpy as np
import os
import time 

from ViveServer.Client import Client
from Motion import Motion

a_arr = []
b_arr = []


class GPS(object):
    def __init__(self):
        self.x = 0          # x-center of robot in m
        self.y = 0          # y-center of robot in m
        self.theta = 0      # angle from x-axis that robot is facing in degrees, -180 < self.theta < +180
        
        self.v = 0          # linear velocity in mm/sec, optional attribute
        self.w = 0          # angular velocity in rad/s, opitional attribute

        self.center_x = 0   # x-center between trackers 3 and 4, creating reference x-axis in m
        self.center_y = 0   # y-center between trackers 3 and 4, creating reference y-axis in m
        self.ref_angle = 0  # angle of new reference axes in relation to SteamVR axes


    #### Write array to filename ####
    def output_arr(self, array, filename="test_navrobot.txt"):
        with open(filename,'w') as f:
            for item in array:
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
        Cx = ((Bx - Ax) / 2) + Ax
        Cy = ((By - Ay) / 2) + Ay

        r = 0.05        # assuming the center of the robot is exactly 50mm from the wheel axis
        # offset to circle at center Cx,Cy
        self.x = Cx - (r*np.cos(np.radians(self.theta)))
        self.y = Cy - (r*np.sin(np.radians(self.theta)))


    #### Update GPS object's x, y, and theta ####
    def get_robot_pose(self, c):
        gps = c.get_trackers_coordinates()
        a = list(gps['tracker_1'])
        b = list(gps['tracker_2'])
        a_arr.append(a[0:3])
        b_arr.append(b[0:3])
        self.tracker_to_xytheta(a[0],a[2],b[0],b[2])   # grab current pose of robot


    #### Find reference axes and angle ####
    def ref_axes(self, client):
        gps = client.get_trackers_coordinates()
        c = list(gps['tracker_3'])
        d = list(gps['tracker_4'])
        x1, y1 = c[0], c[2]
        x2, y2 = d[0], d[2]
        self.center_y = (y2 - y1)/2 + y1
        self.center_x = (x2 - x1)/2 + x1
        self.ref_angle = np.degrees(np.arctan2((y2-y1), (x2-x1)))


    #### Translate tracker coordinates from GPS in relation to new reference axes ####
    def translation(self, new_x, new_y, new_ang):
        orig_point = np.array((new_x, new_y))
        center = np.array((0,0))
        r = np.linalg.norm((center-orig_point))

        theta_n = np.degrees(np.arctan2(new_y, new_x))                              # angle from x-axis to desired coordinates
        trans_ang = 180 + self.ref_angle + new_ang                                  # translated new_ang in reference of GPS x-axis
        if trans_ang > 180:
            trans_ang = trans_ang - 360

        center_x = self.center_x * 1000                                             # convert self.center_x from m to mm
        center_y = self.center_y * 1000                                             # convert self.center_y from m to mm

        trans_x = center_x - (r * np.cos(np.radians(self.ref_angle+theta_n)))       # x coordinate of desired point in relation to GPS axis 
        trans_y = center_y - (r * np.sin(np.radians(self.ref_angle+theta_n)))       # y coordinate of desired point in relation to GPS axis

        return trans_x, trans_y, trans_ang


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
        angle_threshold = 0.5
        while (abs(self.theta-new_ang) > angle_threshold):              # feedback system to fix angle
            tic = time.monotonic()
            temp_ang = self.theta
            self.calc_turn_angle(new_ang, robot, move_robot)
            self.get_robot_pose(c)
            r = 0.05
            self.w = (self.theta - temp_ang) / (time.monotonic() - tic)
            self.v = self.w * r


    #### Calculate dest_angle robot needs to face to travel straight to destination ####
    def dest_angle(self, conv_x, conv_y, new_x, new_y):                 
        dest_ang = np.degrees(np.arctan2(new_y-conv_y, new_x-conv_x))
        x = dest_ang + 180 if dest_ang > 0 else (360+dest_ang) + 180
        if x > 180:
            dest_ang = x - 360
        return dest_ang


    #### Move straight to destination ####
    def move_to_dest(self, d:'distance to move in mm', v=100, w=0, robot=None, c=None):          
        self.v = v                                                                          # in mm/sec
        self.w = w                                                                          # in ang/sec
        t = d/v                                                                             # time it takes to move d mm with v mm/sec
        
        wheelbase = 235
        vL = v - (wheelbase/2)*np.radians(w)
        vR = v + (wheelbase/2)*np.radians(w)

        r = 0.05                                                                            # radius between the two trackers in m
        init_delay = 0.05                                                                   # delay between starting loop and comparison
        start = time.monotonic()        
        while (time.monotonic() + init_delay < start + t):
            p1 = np.array((self.x, self.y))
            tic = time.monotonic()                                                          # time from previous position
            robot.drive_direct(int(np.round(vL)) ,int(np.round(vR)))
            self.get_robot_pose(c)
            p2 = np.array((self.x, self.y))
            self.v = (np.linalg.norm(p2-p1)/(time.monotonic()-tic))*1000
            self.w = (r * ((np.linalg.norm(p2-p1))/(time.monotonic()-tic))) / (np.power(r,2))
        
        robot.drive_direct(0,0)


    #### Main function to use to travel robot to a specific coordinate ####
    def travel(self, robot, c, des_x:'x-coordinate in m', des_y:'y-coordinate in m', des_ang:'final angle (in degrees), range: -180<x<+180', other_trackers=False): 
        move_robot = Motion.RobotTranslator(robot)
        self.get_robot_pose(c)      # get coordinates of robot if not taken already

        # Convert desired xy from m to mm
        des_x = des_x * 1000
        des_y = des_y * 1000
        
        if other_trackers:
            self.ref_axes(c)            # calculate reference axes in relation to trackers 3 and 4
            print("New center: ", self.center_x, self.center_y, "Reference angle: ", self.ref_angle)
            new_x, new_y, new_ang = self.translation(des_x, des_y, des_ang)
        else:
            new_x = des_x
            new_y = des_y
            new_ang = des_ang

        p2 = np.array((new_x, new_y))

        #### STEPS TO GET TO DESTINATION ####
        # 1. First, have robot face destination
        # 2. Second, have the robot start moving forward to destination (move towards minimum distance between the two points)
        # 3. Check if we're at destination, else restart steps 1 and 2
        # 4. Once at destination (or close enough within threshold), rotate robot to final angle
        
        print("Moving to target location: (" , des_x , "," , des_y , ") with final angle: ", des_ang)
        
        # 3. repeat steps 1 and 2 if needed
        lin_threshold = 10      # difference between desired coordinate and measured coordinate that is acceptable
        while (abs(new_x - (self.x*1000)) >= lin_threshold or abs(new_y - (self.y*1000)) >= lin_threshold):
            # Get current robot's coordinates in mm
            self.get_robot_pose(c)
            conv_x = self.x * 1000
            conv_y = self.y * 1000
            p1 = np.array((conv_x, conv_y))

            # 1. rotate to destination)
            dest_ang = self.dest_angle(conv_x, conv_y, new_x, new_y)
            self.turn_to_angle(c, dest_ang, robot, move_robot)

            # 2. move to destination
            dist = np.linalg.norm(p2-p1)                    # in mm
            self.move_to_dest(d=dist, robot=robot, c=c)

        # 4. rotate to final angle at destination
        self.turn_to_angle(c, new_ang, robot, move_robot)
        
        # write gps data to text file
        # out_arr = np.vstack((a_arr,b_arr))
        # self.output_arr(arr=out_arr)
        
