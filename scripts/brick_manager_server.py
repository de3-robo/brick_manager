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
    new = False

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
    resp.z = p[2] - 0.13 #Offset for table height
    resp.wx = p[3]
    resp.wy = p[4]
    resp.wz = p[5]
    return resp

brick_manager_s = rospy.Service('get_pick_loc', QueryBrickLoc, brick_manager_server)
goal_manager_s = rospy.Service('get_place_loc', QueryBrickLoc, goal_manager_server)

rospy.spin()
