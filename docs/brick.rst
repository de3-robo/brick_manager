Brick Manager Server
========================

Brick Manager is the *ROS Package* which tells Arm Master where it should be placing each of the bricks. As a brief summary
the function being used asks for the number of the brick being placed, finds the coordinates and orientation the end effector needs to place that brick and
sends it back to Arm Master.

This package is run from the python file ``brick_manager_server.py`` and this will be the focus of explanation. There is, however, a second script that was not implemented in the end,
``SamRowan.py`` and this will also be briefly covered.

File Structure
-----------------
The core of the package files are structure as follows::

brick_manager
├── docs
└── scripts
    ├── brick_manager_server.py
    └── samrowan.py
├── CMakeLists.txt
└── package.xml


The package itself is incredibly simple. Within the scrips section only two python scripts exist, ``brick_manager_serve.py`` and ``samrowan.py``; both of these will be discussed later on.


brick_manager_server
-----------------------------------

The first function, ``brick_manager_server``, is the intermediary service between Arm Master and Brick Manager. This function takes the input of whatever
brick Arm Master wants next, and relays back the coordinate and angular positions of the end effector when it places the brick. This function can be seen below::

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

The content of this function can be found in each of the three coordinate functions. However, it is no longer used as a standalone function.

The Code's Functions
-----------------------------------

Within brick_manager_server there are three functions which could be used to generate the brick. In summary the first of these runs from a hardcoded list of positions
whilst the second and third generate the wall based on a predefined base width and number of bricks.

The First Function: goal_manager_server
-----------------------------------

This first set of brick positions was generated so that the other packages could be tested whilst this one was developed. In essence it is a pre-defined set of
positions which gets run through in increments of one, as each brick is successfully placed. This 'brick number' is iterated through in the main loop of Arm Master and received through a service in Brick Manager.

The function will be discussed in two halves; the start which has an if statement to return the correct brick positions, and the end which uses ``brick_manager_server``and returns the values to Arm Manager.
The start is as follows::

    def goal_manager_server(req):

        num = req.num                                   # establishing which number was requested

        if num == 0:                                    # if statement to iterate through brick positions
            p = [0.6, 0, 0.116, 0, 0, 1.57]
        elif num == 1:
            p = [0.6, -0.2-0.05, 0.116, 0, 0, 1.57]
        elif num == 2:
            p = [0.6, 0.2+0.05, 0.116, 0, 0, 1.57]
        elif num == 3:
            p = [0.6, -0.18, 0.176, 0, 0, 1.57]
        elif num == 4:
            p = [0.6, +0.18, 0.176, 0, 0, 1.57]

The above code is an extract of the function initialisation and the first few brick positions. As you can see, the function receives a value, which is the 'place number'
from Arm Master. This is evaluated as a number in line 3 of the extract and the if statement proceeds to select the correct coordinate position.

The second halve, and end of the function, uses the previously discussed brick_manager_server content and creates an output that will be sent back
to the Arm Master main code via the defined message type, which is ``QueryBrickLoc()``. ``get_pick_loc`` and ``get_place_loc`` are what Arm Master is
calling for and is what allows the position to be returned to Arm Master.

Finally, ``rospy.spin()`` prevents the python script from closing and allows the packages to continue running. The code can be seen below::

        resp = QueryBrickLocResponse()
        resp.x = p[0]
        resp.y = p[1]
        resp.z = p[2]
        resp.wx = p[3]
        resp.wy = p[4]
        resp.wz = p[5]
        return resp

    brick_manager_s = rospy.Service('get_pick_loc', QueryBrickLoc, brick_manager_server)
    goal_manager_s = rospy.Service('get_place_loc', QueryBrickLoc, goal_manager_server)

    rospy.spin()

The Second Function: goal_manager_server2
-----------------------------------

The second function works much the same way as the first; it receives the brick number from Arm Master and returns the positions in the same way.
However, this code generates it's positions based off an algorithm instead of pre-defined locations.

This section will simply talk about the generation algorithm, since the implementation is the same. The first section of the algorithm is shown bellow::

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

    input_nos = 15                  #input the number of bricks in wall
    input_width = 5                 #input the number of bricks long the wall will be
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

This section of code could have been broken into smaller chunks to explain, but it is in essence incredibly simple. All this section does
is set the definitions for a start position, the geometry of the brick, establish the brick start position as an array, initiate an empty list to be built upon,
establish the size the wall will be built to and give tolerances to the brick positions so they do not touch and will therefore not interfere when placed. Whilst
this does sound like a lot it is all simply definitions for the generative algorithm.

The generative for loop can be seen below::

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

This for loop runs through itself until it has looped for as many cycles as bricks needed. The if statements within this loops are designed so that the robot will build a corner. This corner will always be
built away from the starting brick position and this direction is controlled by the first if statement.

The second if statement gives the offset between each layer, so that the bricks are slightly skewed, like in a standard brick wall.

The Third Function: goal_manager_server3
-----------------------------------

The final function, ``goal_manager_server3`` works in the same way as ``goal_manager_server2`` in the sense it generates the wall coordinates as it goes.
This section will simply talk about the differences in generation technique and the resulting shape.

The generative for loop is shown below::

    for i in range(0, square_number):

        pyramid_seperate.append([round(bstart[0]+xnos*blength+offset*(blength/2), 3),bstart[1],round(bstart[2]+znos*bheight, 3),bstart[3],bstart[4],bstart[5]])

        if xnos < wall_base_width-1:
            xnos += 1
        else:
            xnos = 0
            znos += 1
            offset += 1

This loop is simpler than the second function and simply generates a parallelogram of points. This is not enough however since the point of this function was to create a pyramid of points.
To solve this the following set of if statements break the list up and 'peel' away the ends of each level. Once this is done each new list is appeneded to the end of the last to create a new
list of points in the correct order. These statements are::

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

These statements first ask for the height of the wall, to determine how many items need to be removed. Once this is done it completes the actions mentioned before and removes unwanted overhanging members.

SamRowan.py
-----------------------------------

This class is currently obsolete and is no longer used. However, it does contain the initial generative algorithms for the coordinate points. An example of this is the function ``generate_simple_wall`` which simple creates towers of bricks
next to each other. The aim of the functions in this python file was to create a coordinate generation system which could adapt to any task space by always placing the bricks in an open space::

    if ypickup >= 0:
            xstart = -0.5
            ystart = -0.5
        else:
            xstart = -0.5
            ystart = 0.5

The above code is an extract from ``generate_simple_wall``. This piece of code considered where the brick was being picked up from and, since the original code built in the positive
x direction, it placed the first brick at a negative x coordinate and a y coordinate which had a value opposite to the pickup position. This position was always 0.5, but on the opposite side.

This system was made obsolete since 180 degree rotations around the robot caused it to try and go through itself, which would cause damage to the robot.
