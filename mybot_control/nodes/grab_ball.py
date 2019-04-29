#!/usr/bin/env python

import sys
import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import fabs
import numpy as np
import math

min_dist_ball = 0.8
min_dist_wall = 1.5

class Grabber():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.crossbar_publisher = rospy.Publisher('/mybot/crossbar_position_controller/command', Float64, queue_size=10)
        rospy.sleep(1)
        self.status = "lifting_bar"
        self.substatus = "unknown"
        self.delta_angle = None
        self.radius = 0
        self.see_red = False
        self.covered = False
        self.scan = None
        self.view = {
                'right': float('Inf'),
                'fright': float('Inf'),
                'front_narrow': float('Inf'),
                'front': float('Inf'),
                'fleft': float('Inf'),
                'left': float('Inf')
        }
        self.navigate()
    def scan_callback(self, data):
        self.scan = data
        
    def get_ball_angle_dist(self,min_angle,max_angle):

        ar = self.scan.ranges
        l = len(ar)
        inc = 4
        try:
            for i in range(l):
                angle = 360.0/l*i -90
                if((angle < min_angle)or(angle > max_angle)):
                    continue
                dist_c = np.mean(ar[i-inc:i+inc])
                if(not (dist_c < 10)):
                    continue
                theta = math.atan(0.2/(0.2+dist_c))
                itr = int(abs(theta/self.scan.angle_increment)) + 10
                half_itr = int(itr/2.0)
                dist_b = np.mean(ar[i-half_itr-inc:i-half_itr+inc])
                dist_a = np.mean(ar[i-itr-inc:i-itr+inc])
                if(i + half_itr + inc > l):
                    lower = 0
                    upper = i+half_itr + inc-l
                else:
                    lower = i+half_itr - inc
                    upper = i + half_itr + inc
                dist_d = np.mean(ar[lower:upper])
                if(i+ itr + inc > l):
                    lower = 0
                    upper = i + itr + inc -l
                else:
                    lower = i + itr - inc
                    upper = i + itr + inc
                dist_e = np.mean(ar[lower:upper])
                if(dist_c < 10.0):
                    if((dist_b - dist_c < 0.06) and (dist_d-dist_c < 0.06)):
                        #print((dist_c+0.2)/math.cos(theta))
                        val = (dist_c+0.2)/math.cos(theta)*0.9
                        if(((dist_a - dist_c)>val) and (dist_e-dist_c)>val):
                            ang = self.scan.angle_min + i*self.scan.angle_increment
                            dist = dist_c
                            #print(str(ang/3.14*180) + " " + str(dist))
                            return ang,dist
                    
                #print(str(dist_a) + " " +str(dist_b) + " " +str(dist_c) + " " +str(dist_d)  + " " + str(dist_e))
               
        except Exception as error: 
            print(error)
            return None

        
        
        
        

    def update_substatus_when_searching(self):
        if self.view['fleft'] <= min_dist_wall and self.view['fright'] <= min_dist_wall and self.view['front'] <= min_dist_wall:
            self.change_substatus("turning left")
        elif self.view['fleft'] > min_dist_wall and self.view['fright'] <= min_dist_wall and self.view['front'] <= min_dist_wall:
            self.change_substatus("turning left")
        elif self.view['fleft'] <= min_dist_wall and self.view['fright'] > min_dist_wall and self.view['front'] <= min_dist_wall:
            self.change_substatus("turning left")
        elif self.view['fleft'] <= min_dist_wall and self.view['fright'] <= min_dist_wall and self.view['front'] > min_dist_wall:
            self.change_substatus("forwarding")
        elif self.view['fleft'] <= min_dist_wall and self.view['fright'] > min_dist_wall and self.view['front'] > min_dist_wall:
            self.change_substatus("turning left")
        elif self.view['fleft'] > min_dist_wall and self.view['fright'] <= min_dist_wall and self.view['front'] > min_dist_wall:
            self.change_substatus("following")
        elif self.view['fleft'] > min_dist_wall and self.view['fright'] > min_dist_wall and self.view['front'] <= min_dist_wall:
            self.change_substatus("turning left")
        elif self.view['fleft'] > min_dist_wall and self.view['fright'] > min_dist_wall and self.view['front'] > min_dist_wall:
            if self.substatus == "unknown":
                self.change_substatus("forwarding")
            else:
                self.change_substatus("groping convex corner")

    def command_bot(self, lx, ly, lz, ax, ay, az):
        twist = Twist()
        twist.linear.x = lx; twist.linear.y = ly; twist.linear.z = lz
        twist.angular.x = ax; twist.angular.y = ay; twist.angular.z = az
        self.cmd_pub.publish(twist)
    def lift_crossbar(self):
        self.crossbar_publisher.publish(3.14)
        rospy.sleep(1)
        self.status = "acquiring_ball"
    def lower_crossbar(self):
        self.crossbar_publisher.publish(0.0)
        rospy.sleep(1)
        self.status = "retrieving"

    def change_status(self, input_status, info=None):
        if input_status != self.status:
            print "current: ", self.status, "   next: ", input_status
            self.status = input_status

    def change_substatus(self, input_substatus, info=None):
         if input_substatus != self.substatus:
            print "current sub: ", self.substatus, "   next sub: ", input_substatus
            self.substatus = input_substatus

    def init(self):
        if self.covered:
            self.change_status("terminated")
        else:
            self.change_status("searching")

    def search(self):
        if self.see_red:
            print "see red"
            self.change_status("moving_to_red")
        elif self.covered:
            self.change_status("terminated")
        else:
            if self.substatus == "unknown":
                return
            elif self.substatus == "forwarding":
                self.forward_search()
            elif self.substatus == "turning left":
                self.turn_left_search()
            elif self.substatus == "turning right":
                self.turn_right_search()
            elif self.substatus == "following":
                self.follow_search()
            elif self.substatus == "groping convex corner":
                self.grope_convex_corner_search()
            else:
                print("unknown sub status under searching")

    def forward_search(self):
        self.command_bot(0.5, 0, 0, 0, 0, 0)

    def turn_left_search(self):
        self.command_bot(0, 0, 0, 0, 0, 0.5)

    def turn_right_search(self):
        self.command_bot(0, 0, 0, 0, 0, -0.5)

    def follow_search(self):
        self.command_bot(0.5, 0, 0, 0, 0, 0.07)

    def grope_convex_corner_search(self):
        self.command_bot(0.5, 0, 0, 0, 0, -0.3)

    def move_to_red(self):
        # skew and far
        if fabs(self.delta_angle) > 5 and self.radius < 50:
            linear = min(0.5, self.view["front_narrow"]*0.2)
            angular = min(0.5, self.delta_angle*0.01)
        # central and far
        elif fabs(self.delta_angle) <= 5 and self.radius < 50:
            linear = min(0.5, self.view["front_narrow"]*0.2)
            angular = 0
        # skew and near
        elif fabs(self.delta_angle) > 5 and self.radius >= 50:
            if self.view["front_narrow"] <= min_dist_ball:
                linear = 0
            else:
                linear = min(0.2, self.view["front_narrow"]*0.1)
            angular = min(0.5, self.delta_angle*0.05)
        # central and near
        else:
            if self.view["front_narrow"] <= min_dist_ball:
                linear = 0
                self.change_status("terminated")
            else:
                linear = min(0.2, self.view["front_narrow"]*0.1)
            angular = 0
        self.command_bot(linear, 0, 0, 0, 0, angular)


    def grab(self):
       pass

    def retrieve(self):
        print("in retrieving...")
        self.command_bot(0.0,0,0,0,0,0)

    def move_to_src(self):
        pass

    def terminate(self):
        self.command_bot(0, 0, 0, 0, 0, 0)

    def navigate(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            print(self.status)
            if self.status == "init":
                self.init()
            elif self.status == "searching":
                self.search()
            elif self.status == "moving_to_red":
                self.move_to_red()
            elif self.status == "lifting_bar":
                self.lift_crossbar()
            elif self.status == "lowering_bar":
                self.lower_crossbar()
            elif self.status == "acquiring_ball":
                self.acquire_ball()
            elif self.status == "grabbing":
                self.grab()
            elif self.status == "retrieving":
                self.retrieve()
            elif self.status == "moving_to_src":
                self.moving_to_src()
            elif self.status == "terminated":
                self.terminate()
            else:
                print("unknown status")
            rate.sleep()
    
    #def position(self,ang_error,dist):
        
    def print_scan(self):
        print(self.scan.angle_min)
        print(self.scan.angle_max)
        for i,r in enumerate(self.scan.ranges):
            print(str(i) + "  " + str(r) + "  " + str(type(r)))
        print("__________________________________")
    
    def get_simple_angle(self,alpha):
        new_alpha = alpha
        if (abs(alpha) > 2*math.pi):
            if(alpha < 0):
                new_alpha = -(-alpha)%(2*math.pi)
            else:
                new_alpha = alpha%(2*math.pi)
        else:
            if(alpha < 0):
                new_alpha = (2*math.pi + alpha)
            else:
                new_alpha = alpha
        if(new_alpha > math.pi):
            new_alpha = -(math.pi - (new_alpha - math.pi))
        return new_alpha
    
    K_angle = 0.5
    K_lin = 0.1
    final_dist = 0.05
    def acquire_ball(self):
        ret = self.get_ball_angle_dist(-90,90*3)
        if(not (ret == None)):
            ang = ret[0]
            dist = ret[1]
            if(dist < 0.25):
                self.command_bot(0,0,0,0,0,0)
                print("dist = " + str(dist))
                self.status = "lowering_bar"
                return
            print("ang = " + str(self.get_simple_angle(ang)/3.14*180))
            ang = self.get_simple_angle(ang) + 3.25 #not 3.14 to avoid systematic error
            ang_vel = self.K_angle*self.get_simple_angle(ang)
            #print("angle = " + str(self.get_simple_angle(ang)))
            #print("simple angle = " + str(self.get_simple_angle(ang)/3.14*181))
            #print("ang  = " + str(self.get_simple_angle(ang)/3.14*180))
            dist_error = dist - self.final_dist
            if(abs(self.get_simple_angle(ang)) < 10.0/180.0*3.14):
                lin_vel = -self.K_lin*dist_error
            else:
                lin_vel = 0.0
            self.command_bot(lin_vel,0,0,0,0,ang_vel)
            #print("ang error........")
            #print(ang_error)
            #print(ang_vel)
            rospy.sleep(0.5)
        

def main(args):
    rospy.init_node('grabber', anonymous=True)
    rospy.sleep(1)
    try:
        grabber = Grabber()

    except KeyboardInterrupt:
        print("grabber Shutting down")

if __name__ == '__main__':
    main(sys.argv)
