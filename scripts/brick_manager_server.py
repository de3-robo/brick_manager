#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import tf
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64
from moveit_commander.conversions import pose_to_list
from de_msgs.srv import QueryBrickLoc, QueryBrickLocResponse
from samrowan import SamRowan
# from keith_code import *

# Provides the goal location queries.

rospy.init_node('brick_manager_server')


#Create Classes to Manager Goal and Brick Stack Location

GoalManager = SamRowan(5,4)

def brick_manager_server(req):
    #num = req.num #change req to req.num to do placed iteration thing
    resp = QueryBrickLocResponse()
    p = [0.5, 0.5, 0.116, 0, 0, 1.57] # z+0.2?
    resp.x = p[0]
    resp.y = p[1]
    resp.z = p[2]
    resp.wx = p[3]
    resp.wy = p[4]
    resp.wz = p[5]
    return p


def goal_manager_server(req):

    num = req.num

    #p, new = GoalManager.get_next_goal_loc(num)    #SAM DO CODE AND LOGIC IN HERE #Start with one value in here to avoid errors
    #     # while not new: #Query untill you get a new position
    #     # rospy.sleep(0.05)
    """
    print("GoalManager P: ", p)

    which_brick = QueryBrickLocRequest()
    print("placed:", which_brick)
    """

# #########################################################################
# ###BELOW CODE IS THE ORIGINAL####################
# ############################################################################
    if num == 0:
        p = [0.6, 0, 0.116, 0, 0, 1.57]
    elif num == 1:
        p = [0.6, -0.2-0.05, 0.116, 0, 0, 1.57]
    elif num == 2:
        p = [0.6, 0.2+0.05, 0.116, 0, 0, 1.57]

    elif num == 3:
        p = [0.6, -0.18, 0.176, 0, 0, 1.57]
    elif num == 4:
        p = [0.6, +0.18, 0.176, 0, 0, 1.57]

    elif num == 5:
        p = [0.6, 0, 0.236, 0, 0, 1.57]
    elif num == 6:
        p = [0.6, -0.2-0.05, 0.236, 0, 0, 1.57]
    elif num == 7:
        p = [0.6, 0.2+0.05, 0.236, 0, 0, 1.57]

    elif num == 8:
        p = [0.6, -0.18, 0.296, 0, 0, 1.57]
    elif num == 9:
        p = [0.6, +0.18, 0.296, 0, 0, 1.57]

    elif num == 10:
        p = [0.6, 0, 0.356, 0, 0, 1.57]
    elif num == 11:
        p = [0.6, -0.2-0.05, 0.356, 0, 0, 1.57]
    elif num == 12:
        p = [0.6, 0.2+0.05, 0.356, 0, 0, 1.57]

    resp = QueryBrickLocResponse()
    resp.x = p[0]
    resp.y = p[1]
    resp.z = p[2] #- 0.13 #Offset for table height
    resp.wx = p[3]
    resp.wy = p[4]
    resp.wz = p[5]
    return resp

brick_manager_s = rospy.Service('get_pick_loc', QueryBrickLoc, brick_manager_server)
goal_manager_s = rospy.Service('get_place_loc', QueryBrickLoc, goal_manager_server)

rospy.spin()

def goal_manager_server2(req):
    num = req.num
    new = False

    import math
    pi = math.pi

    #pose of first brick in wall, cornerstone
    xstart=1
    ystart=-0.8
    zstart=0.116
    xtheta=0
    ytheta=0
    ztheta= pi/2

    #geometry of the brick
    blength = 0.2
    bwidth = 0.09
    bheight = 0.06
    angle = pi/2  #90 degree rotation

    #puts pose into the list
    bstart=[xstart,ystart,zstart,xtheta,ytheta,ztheta]

    #initiates list that will contain all the brick poses
    pos_list = []   #initate the list

    input_nos = 15       #input the number of bricks in wall
    input_width = 5       #input the number of bricks long the wall will be
    brick_number = int(input_nos)
    width = int(input_width)
    width1= math.ceil(width/2)
    shift1=(blength+bwidth)/2
    shift2=(blength-bwidth)/2
    round_up = brick_number

    #initating counters to help design the wall
    xnos = 0
    znos = 0

    round_up = brick_number


    #adding tolerances to the wall design
    tol = 0.02
    blength = blength + tol
    bheight = bheight + tol


    #building the list of brick poses

    for i in range(1, round_up+1):

        if xnos <= width/2 and (znos %2) == 1:
            pos_list.append([bstart[0]-xnos*blength,bstart[1],bstart[2]+znos*bheight,bstart[3],bstart[4],bstart[5]]) #edit this so the alignment is always correct
        elif xnos > width/2 and (znos %2) == 1:
            pos_list.append([bstart[0]+(blength-bwidth)/2,bstart[1]-(xnos-math.floor(width/2))*blength+ (blength+bwidth)/2,bstart[2]+znos*bheight,bstart[3],bstart[4],bstart[5]+angle]) #edit this so the alignment is always correct
        elif xnos <= width/2 and (znos %2) == 0:
            pos_list.append([bstart[0]-xnos*blength+bwidth,bstart[1],bstart[2]+znos*bheight,bstart[3],bstart[4],bstart[5]]) #edit this so the alignment is always correct
        elif xnos > width/2 and (znos %2) == 0:
            pos_list.append([bstart[0]+(blength-bwidth)/2,bstart[1]-(xnos-math.floor(width/2))*blength-bwidth+ (blength+bwidth)/2,bstart[2]+znos*bheight,bstart[3],bstart[4],bstart[5]+angle]) #edit this so the alignment is always correct

        if xnos < width:
            xnos+=1
        else:
            xnos=0
            znos+=1

    p= pos_list
    resp = QueryBrickLocResponse()
    resp.x = p[num][1]
    resp.y = p[num][0]
    resp.z = p[num][2] #Offset for table height
    resp.wx = p[num][3]
    resp.wy = p[num][4]
    resp.wz = p[num][5]
    return resp

def goal_manager_server3(req):
    num = req.num
    new = False

    import math
    pi = math.pi

    xpickup=0.5
    ypickup=0.5
    xpicktheta=0
    ypicktheta=0
    zpicktheta=0

    if ypickup >= 0:
        xstart = -0.5
        ystart = -0.5
    else:
        xstart = -0.5
        ystart = 0.5

    zstart=0.2 #add a small offset
    xtheta= 3.14 #THIS MEANS THE GRIPPER WILL BE FACING downward
    ytheta=0
    ztheta= 3.14/4 #AT THIS VALUE THE GRIPPER IS straight with respect to base

    blength = 0.2+0.005                                                       #geometries of the brick
    bwidth = 0.09+0.005
    bheight = 0.062

    bstart=[xstart,ystart,zstart,xtheta,ytheta,ztheta]     #first brick position mirrors position of where we place the bricks
    pos_pyramid_final = []

    xnos = 0
    znos = 0

    input_width = input()
    input_width = 7 #Pass in from class variable
    wall_base_width = int(input_width)

    input_height = input()
    input_height = 5 #preset for now
    wall_height = int(input_height)

    pyramid_seperate = []
    offset = 0

    square_number = wall_height*wall_base_width

    for i in range(0, square_number):

        pyramid_seperate.append([round(bstart[0]+xnos*blength+offset*(blength/2), 3),bstart[1],round(bstart[2]+znos*bheight, 3),bstart[3],bstart[4],bstart[5]])

        if xnos < wall_base_width-1:
            xnos += 1
        else:
            xnos = 0
            znos += 1
            offset += 1

    if wall_height == 1:
        pos_final = pyramid_seperate
    elif wall_height == 2:
        pos_final = pyramid_seperate[:(wall_base_width*2)-1]
    elif wall_height == 3:
        pyr_one = pyramid_seperate[:(wall_base_width*2)-1]
        pyr_two = pyramid_seperate[(wall_base_width*2):(wall_base_width*3)-2]
        pos_final = pyr_one + pyr_two
    elif wall_height == 4:
        pyr_one = pyramid_seperate[:(wall_base_width*2)-1]
        pyr_two = pyramid_seperate[(wall_base_width*2):(wall_base_width*3)-2]
        pyr_three = pyramid_seperate[(wall_base_width*3):(wall_base_width*4)-3]
        pos_final = pyr_one+pyr_two+pyr_three
    else:
        pyr_one = pyramid_seperate[:(wall_base_width*2)-1]
        pyr_two = pyramid_seperate[(wall_base_width*2):(wall_base_width*3)-2]
        pyr_three = pyramid_seperate[(wall_base_width*3):(wall_base_width*4)-3]
        pyr_four = pyramid_seperate[(wall_base_width*4):(wall_base_width*5)-4]
        pos_final = pyr_one + pyr_two + pyr_three + pyr_four

    brick_number = len(pos_final)

    p= pos_final
    resp = QueryBrickLocResponse()
    resp.x = p[num][0]
    resp.y = p[num][1]
    resp.z = p[num][2] #Offset for table height
    resp.wx = p[num][3]
    resp.wy = p[num][4]
    resp.wz = p[num][5]
    return resp

brick_manager_s = rospy.Service('get_pick_loc', QueryBrickLoc, brick_manager_server)

goal_manager_s = rospy.Service('get_place_loc', QueryBrickLoc, goal_manager_server)


rospy.spin()
