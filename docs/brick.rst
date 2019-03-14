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

real_panda
-----------------------------------

At the start of the runnable code, a variable ``real_panda`` is defined::

    # ----------------------------------------------
    real_panda = True
    # ----------------------------------------------

It is essential this is set correctly depending on weather you wish to run the code on *Gazebo* or on the real *Franka Panda*.


Setting this variable to ``False``, ensures that the code doesn't wait for services from the real robot which will
not appear on the ROS network

.. warning::
   The ``real_panda`` variable also needs to be set in `move_arm_server.py`_

.. warning::
   If you find that the code is not running and is getting hanged up at launch time it could be because the arm_master node
   is waiting for all the required nodes to be launched. Double check ``real_panda`` is correctly set.

   Note that the code calls ``rospy.wait_for_service()`` each time it is required to connect to another service
   or action client::

        def connect_srv(name, msg_type):

            rospy.loginfo("Searching for " + name + " .... ")
            rospy.wait_for_service(name)
            srv_wrapper = rospy.ServiceProxy(name, msg_type)
            rospy.loginfo(name + " CONNECTED")
            return srv_wrapper


Loop
-----------------------------------
The main loop has the following structure::

  #Continue to loop until you have placed the correct number of bricks
  while not rospy.is_shutdown():
        if placed < num_bricks:

            #Query Positions
            brick = get_brick_pos()
            goal = get_goal_pos()

            #Move towards brick and pick up
            pick_up(brick)

            #Move from pick up positon to place down position
            move_towards(brick, goal)

            #Place down brick
            place_down(goal)

            # Increment num of brick placed
            placed += 1


If you wish to change how the arm moves, change the order in which the ``pick_up()``, ``place_down()``, ``place_down()``
functions are called. Additional motion functions also available in ``arm_master_main.py`` are ``go_to()`` and ``move_arm_curve()``. To illustrate,
The main loop for our project implementation was implemented as follows::

     while not rospy.is_shutdown():  # Main Control Loop for the arm
            if placed < num_bricks:  # Continue to loop until you have placed the correct number of bricks

                # Query Positions
                brick = get_brick_pos(placed)
                goal = get_goal_pos(placed)

                if goal == last_goal:  # same as last time, don't go back
                    continue
                home = get_home_pos()
                over_head = get_over_pos()

                if not real_panda:
                    gen_brick()
                succ = move_towards(home, brick, circle_points)

                # Pick Place operation then return home

                pick_up(brick)
                succ = move_towards(brick, goal, circle_points, check=False)

                if not real_panda: #Functionality to return to brick location if you dropped it.
                    if not succ:
                        brick_via = brick
                        brick_via[2] += 0.2
                        go_to(brick_via)
                        continue #continue, don't increment placed

                place_down(goal)

                succ = move_towards(goal, home, circle_points)
                placed += 1
                last_goal = goal  # placed down now its a last brick

                rospy.loginfo("Placed")
                # Place another brick from stack onto wall

            else:  # When done just wait
                rospy.loginfo("Done, " + str(placed) + " bricks placed")
            rate.sleep()


Behind the Scenes
-----------------------------------

I will now explain more of the theoretical aspects of what happens when a motion function like ``pick_up()`` is called in
``arm_master_main.py``.

Pick Up
++++++++++++++++++++++

The pick up function in full is::

    def pick_up(target, via_offset=0.3):

        global holding_brick  # use global var

        # First Move to point above the pick up location
        via_point = copy.deepcopy(target)
        via_point[2] += via_offset  # Z offset

        move_arm(via_point)  # Move arm to just above goal
        move_arm(target)  # Lower arm down to goal
        # rospy.sleep(0.5) # Play with timming in here to get desired behaviour
        close_gripper()  # Grasp around brick

        holding_brick = True
