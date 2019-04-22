#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
import numpy as np
from std_msgs.msg import Float64



def connect():
    try:
        global slider_publisher,holder_publisher,arm_publisher
        slider_publisher = rospy.Publisher('/mybot/slider_position_controller/command', Float64, queue_size=10)
        holder_publisher = rospy.Publisher('/mybot/holder_position_controller/command', Float64, queue_size=10)
        arm_publisher = rospy.Publisher('/mybot/arm_position_controller/command', Float64, queue_size=10)
    except rospy.ServiceException as e:
        print(e)   
    print("subscribed!") 

#**slider control****    
def lower_slider():
      slider_publisher.publish(-2.0)
def lift_slider():
      slider_publisher.publish(0.0)

#******holder control
def close_holder():
    holder_publisher.publish(-0.78)
def open_holder():
    holder_publisher.publish(0.0)

#******arm control
def lift_arm():
     arm_publisher.publish(0.0) 
def lower_arm():
    arm_publisher.publish(1.57)       

def haha():
    print("yey")