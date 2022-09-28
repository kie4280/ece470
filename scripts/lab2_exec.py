#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
from socket import fromfd
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([141.35, -97.66, 104.86, -98.08, -89.79, 51.39])

# Hanoi tower location 1
Q11 = [132.21*pi/180.0, -56.84*pi/180.0, 123.74*pi/180.0, -157.17*pi/180.0, -90.66*pi/180.0, 45.66*pi/180.0]
Q12 = [132.17*pi/180.0, -65.17*pi/180.0, 122.8*pi/180.0, -147.89*pi/180.0, -90.61*pi/180.0, 45.07*pi/180.0]
Q13 = [132.19*pi/180.0, -72.62*pi/180.0, 120.27*pi/180.0, -137.91*pi/180.0, -90.58*pi/180.0, 45.55*pi/180.0]

# Hanoi tower location 2
Q21 = [148.54*pi/180.0, -60.03*pi/180.0, 131.00*pi/180.0, -160.97*pi/180.0, -90.73*pi/180.0, 62.00*pi/180.0]
Q22 = [148.57*pi/180.0, -69.32*pi/180.0, 129.76*pi/180.0, -150.44*pi/180.0, -90.68*pi/180.0, 61.97*pi/180.0]
Q23 = [149.04*pi/180.0, -77.07*pi/180.0, 127.36*pi/180.0, -140.29*pi/180.0, -90.65*pi/180.0, 62.4*pi/180.0]

# Hanoi tower location 3
Q31 = [168.07*pi/180.0, -59.58*pi/180.0, 129.19*pi/180.0, -159.31*pi/180.0, -90.68*pi/180.0, 81.52*pi/180.0]
Q32 = [168.09*pi/180.0, -68.28*pi/180.0, 128.02*pi/180.0, -149.44*pi/180.0, -90.63*pi/180.0, 81.5*pi/180.0]
Q33 = [168.11*pi/180.0, -75.84*pi/180.0, 125.53*pi/180.0, -139.38*pi/180.0, -90.59*pi/180.0, 81.47*pi/180.0]


thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def suction_callback(msg):
    global digital_in_0
    digital_in_0 = msg.DIGIN
    pass
#global makes it so that the variable can be used anywhere in the program


############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q, digital_in_0

    ### Hint: Use the Q array to map out your towers by location and "height".

#  start and end indexed by Q, user input prompts start and end locations
# move arm goes to moves to a set of angles which is a point the block is located at
# move block moves from any starting location on one tower to the ending location on another tower
#to program for these purposes, we would need to do 7 move blocks, 6 times-one time for each set of tower combinations
    start_point = Q[start_loc][start_height-1]
    end_point = Q[end_loc][end_height-1]
    move_arm(pub_cmd, loop_rate, start_point, 4.0, 1)
    gripper(pub_cmd,loop_rate,suction_on)
    time.sleep(0.5)
    if digital_in_0 == 0:
         move_arm(pub_cmd, loop_rate, home, 4.0, 1)
         raise RuntimeError("Missing block")
    move_arm(pub_cmd, loop_rate, home, 4.0, 1)
    move_arm(pub_cmd, loop_rate, end_point, 4.0, 1)
    gripper(pub_cmd, loop_rate, suction_off)
    move_arm(pub_cmd, loop_rate, home, 4.0, 1)

    error = 0



    return error


############### Your Code End Here ###############
block_count = [0,0,0]


def TowerOfHanoi(n, from_rod, to_rod, aux_rod, pub_command, loop_rate):
    global block_count
    if n == 0:
        return
    TowerOfHanoi(n-1, from_rod, aux_rod, to_rod, pub_command, loop_rate)
    #n/N - blocks left to move
    block_count[to_rod] += 1
    start_height = block_count[from_rod] if block_count[from_rod] > 0 else 1
    end_height = block_count[to_rod] if block_count[to_rod] > 0 else 1
    # suction detection goes here
    move_block(pub_command, loop_rate, from_rod, start_height, to_rod, end_height)

    block_count[from_rod] -= 1
    print("height", start_height, end_height)
    print("block_count", block_count)
    
    print("Move disk", n, "from rod", from_rod, "to rod", to_rod)
    TowerOfHanoi(n-1, aux_rod, to_rod, from_rod, pub_command, loop_rate)
 
 

def main():

    global home
    global Q
    global SPIN_RATE
    global block_count

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_suction = rospy.Subscriber('ur3/gripper_input', gripper_input, suction_callback)
    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input


    start_pos = int(input("Enter number of starting pos <Either 1 2 3 or 0 to quit> "))
    end_pos = int(input("Enter number of ending pos <Either 1 2 3 or 0 to quit> "))



    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input



    # Driver code
    N = 3
    aux_pos = 5 - start_pos - end_pos
    block_count[start_pos-1]=3
    
    # A, C, B are the name of rods
    TowerOfHanoi(N, start_pos-1, end_pos-1, aux_pos,pub_command, loop_rate)

#N - height

    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)





    ############### Your Code End Here ###############


def test():
    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_suction = rospy.Subscriber('ur3/gripper_input', gripper_input, suction_callback)
    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input


    start_pos = int(input("Enter number of starting pos <Either 1 2 3 or 0 to quit> "))
    end_pos = int(input("Enter number of ending pos <Either 1 2 3 or 0 to quit> "))



    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    move_arm(pub_command, loop_rate, Q[start_pos-1][end_pos-1], 4, 1)


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as e:
        print(e)
