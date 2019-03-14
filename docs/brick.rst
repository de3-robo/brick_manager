Brick Manager Server
========================

*"I am the master of my fate: I am the captain of my soul."*

*- William Ernest Henley*

Arm Master is the *ROS Package* which controls the arm. At the most basic level, its purpose is to query goal
end effector poses, and then move the arm from the current position to meet the new goal.

Code for this computation is located in *python* scripts. Some of these scripts exposes them selves as ROS node, but their are also some scripts that simply
provide pure *python* function calls.

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


Launch files `light.launch`_, `panda_one_brick.launch`_, `sim.launch`_ run all necessary ros nodes.
Refer to **Getting Started** for more information on running the code

The *scripts* are structured such that there is a script which defines the ROS related code, and a script which provides ROS free functions. To illustrate observe the difference between
``arm_master_main.py`` and ``arm_master_functions.py``


Main Control Loop
-----------------------------------

Once running, the Panda arm loops through a control sequence defined in `arm_master_main.py`_,
which calls services and publishes to topics defined in ``move_arm_server.py``

To understand what is happening in this control sequence, I will go line by line through the important information and explain
the necessary concepts required.


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
        move_arm(via_point)  # Move back to via point

        return True


It is queried using a target end effector position, set to the location and rotation of the brick to be picked up (defined by a
``[x, y, z, rot_x, rot_y, rot_z]`` list). The second ``via_offset`` parameter determines how high the above the brick the end effector will first travel before
lowering and picking up the brick.

Pictorially the function ``pick_up()`` looks like:

.. figure::  imgs/pick_up.jpg
   :align:   center

Going line by line, first ``move_arm(via_point)`` is called. This calls the function::

    def move_arm(pos):

    msg = MoveArm()
    rospy.loginfo(pos)
    success = move_arm_wrapper(x=pos[0], y=pos[1], z=pos[2], rot_x=pos[3], rot_y=pos[4], rot_z=pos[5])

    return success


Which further provides a wrapper to the service `move_arm`::

    move_arm_wrapper = connect_srv('move_arm', MoveArm)

All arm moment services are defined in ROS node initialized in ``move_arm_server.py``. When a request is sent to the ``move_arm`` service,
the ``move_arm_handler(req)`` function defined inside ``move_arm_server.py`` is called::

    def move_arm_handler(req):

        goal = [req.x,req.y,req.z,req.rot_x,req.rot_y,req.rot_z]
        # goal = [0.5,-0.5,0.5,0,3.14,0]

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.01)

        via_points = plan_cartesian_path(goal,resolution = 1) #res can be changed

        for point in via_points:
            plan = move_arm_a_to_b(point) #
            #Publish this plan at my own speed
            if not real_panda:
                group.execute(plan, wait=False)
                print("EXECUTING PLAN")

                execute(plan)
            else: #Running on real panada
                # plan = slow_down(plan)
                print("EXECUTING PLAN ON REAL ROBOT")

                group.execute(plan, wait=True)

            group.stop()
            group.clear_pose_targets()

        return True



Depending on whether your running on the real robot or *Gazebo*, how the plan is executed changes, but the fundamental planning of the path doesn't.

First a set of end_effector via_points are determined between the current robot position and the goal position. This is done by calling
`` plan_cartesian_path()`` which then calls ``get_via_points()``. ``get_via_points()`` is a function
defined in the ``arm_server_functions.py`` file. ``get_via_points()`` essentially determines the displacement vector between the start and goal
position and then samples points along the same direction incrementally at a set resolution. Pictorially the operation is as follows:

.. figure::  imgs/get_via_points.jpg
   :align:   center

While much of this sampling computation can be accomplished using the the ``compute_cartesian_path()`` function, ``get_via_points()`` gives up additional flexibility and control over the position
of way points, and is used to break up the movement into smaller chunks. Once the ``via_points`` have been obtained, the next step is to create a robot trajectory which goes through all the points.
This is done using the ``move_arm_a_to_b()`` function::

    def move_arm_a_to_b(goal): #move very short distance


        rospy.loginfo('goal')

        waypoints = []
        wpose = group.get_current_pose().pose
        wpose.position.x += 0.0001
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x = goal[0]
        wpose.position.y = goal[1]  # First move up (z)
        wpose.position.z = goal[2]  # and sideways (y)
        quaternion = tf.transformations.quaternion_from_euler(goal[3], goal[4], goal[5]) #(roll, pitch, yaw)
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]
        waypoints.append(copy.deepcopy(wpose))

        group.set_planning_time(4)
        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.02,        # eef_step
                                           2)         # jump_threshold
        # rospy.loginfo(goal)

        return plan


The ``move_arm_a_to_b()`` function utilises *MoveIt* to solve IK along the desired path. First it gets the current position of the robot from the move group interface::

 wpose = group.get_current_pose().pose

It then reads the desired end effector position, passed in as ``goal``, which is defined with euler angles (``[x, y, z, roll, pitch, yaw]``), and changes it to a quaternion representation
(``[x, y, z, X, Y, Z, W]``). The quaternion representation is equivalent to the euler angles, but rather then represent a rotation with 3 separate rotations around
linearly independent axis, a 4D vector is used. This 4D vector has advantages in that it doesn't degenerate and reach singularities in certain rotation sequences, and
thus can be seen as more general. That said, it is not intuitive to work with quaternion's. All poses in our code-base are encoded with the Euler description and transformed to
a quaternion at the last moment using the ``tf.transformations`` function.

Now that the goal pose is described in the same vector space as the current position (7D vector), a linear interpolation can be calculated between the two. For this purpose,
the ``compute_cartesian_path`` function is used. This function first samples points along the straight line between the waypoints, the distance between the points is given by the
``eef_step`` parameter. It then solves IK for each of those sampled points. As the robot is redundant (7 DOF for a task which requires at most 6 DOF), it is able to find many solutions to
the IK problem. Mathematically this means that that null space of the Jacobean contains vectors other than the zero vector. Redundancy resolution is specified such that IK solution minimises the distance from the
previous solution. Specifically we specify in the ``jump_threshold`` parameter that the difference in joint angles in neighbouring IK solutions can be no greater than 2 radians.

The resulting output is a series of joint angles which describe an arm trajectory along the line between the current and goal position.


.. note::

    Because we always specify the end effector of the robot to be pointing downwards, it remains pointing downwards during the interpolation between the current and
    goal position. Just the x, y, z position of the end effector changes.

.. note:: All poses are taken with respect to a static world frame located at the base of the Randa robot

Focusing back on the ``move_arm_handler(req)`` function. The next step is to execute that path. Regardless of whether the robot is running on *Gazebo* or on the real robot,
the desired joint angles are used to update the set point on the robot's PID controller. This results in a error between current and desired joint angles,
which results in a proportional gain to be applied to the motors, which ultimately moves the arm. Sending all the joint angles in
succession and the arm will track the desired end effector movement. Key parameters here are the frequency at which the joint angles are published and the distance between the joint angles.
As only a feedback is being used to control the robot, extremely large steps in joint angles will lead to un unnatural arm behaviour.

This summarizes the main computation and considerations behind moving the Panda arm. We now focus back on the ``pick_up()`` function. You will see that picking up the brick is simply
a matter of asking the robot arm to move first from its current position to a via point - a set z-offset above the brick. Then to lower down to just above the brick,
close the gripper around the brick, and finally return to the via point. Each time, the motion happens as in the paragraphs described above.

.. note::

    While there are slight differences between controlling the gripper in *Gazebo* vs on the real robot, the essence is to publish
    a desired gripper width to a topic that is being read by a controller on the Franka gripper.


Move towards
+++++++++++++++

``move_towards()`` is the other main motion function called in ``arm_master_main.py``. The mechanics through which the arm moves are identical to the process described in the paragraphs above. After a few layers of functions,
it calls the exact same ``move_arm()`` function. The difference in ``move_towards()`` is how the way way points are selected.
Pictorially what happens when you call the function is as follows:

.. figure::  imgs/move_towards.jpg
   :align:   center


Stepping through the function line by line, you will see exactly how this behaviour is implemented. First the closest points on the circle
to the goal and start location are determined. These will become the entry and exit points to the circle::

    def move_towards(start, end, round_way_points, check=False):

        # find nearest point to pick
        min_start_dist = 10000
        min_start_ind = 0
        min_end_dist = 10000
        min_end_ind = 0

        for key, value in round_way_points.items():
            # print(key, value)
            p = value[0]
            dist_start = distance(start, p)
            dist_end = distance(end, p)

            if dist_start < min_start_dist:
                min_start_dist = dist_start
                min_start_ind = key

            if dist_end < min_end_dist:
                min_end_dist = dist_end
                min_end_ind = key

            print(p)

The circle points are generated by a function ``get_round_points()`` in ``arm_master_functions.py``. By changing the ``res``,
``diameter``, ``height``, ``x_thresh`` the nature of the circle can be changed::

    def get_round_points():

        round_path = dict()
        res = float(20)
        diameter = 1.25
        r = diameter / 2  # diameter of the circle
        height = 0.5  # height of the circle

        x_c = 0
        y_c = 0
        num = 20
        for i in np.arange(num):
            theta = (2 * np.pi) * ((i + 1) / res)
            right, left = get_LR_ind(i)
            neighbour = [right, left]
            x = x_c + r * np.cos(theta)
            y = y_c + r * np.sin(theta)
            pos = [x_c + r * np.cos(theta), y_c + r * np.sin(theta), height]
            round_path[i] = [pos, neighbour]

        x_thresh = -0.2  # x threshold behind the arm

        # remove illegal points
        to_remove = []
        for key, value in round_path.items():
            if value[0][0] < x_thresh:
                r_i, l_i = get_LR_ind(key)

                # go to thoose values and delete your self
                right_neighbour_list = round_path[r_i][1]
                left_neighbour_list = round_path[l_i][1]
                right_neighbour_list.remove(key)
                left_neighbour_list.remove(key)
                to_remove.append(key)

        for i in to_remove:
            # print(i)
            del round_path[i]

        return round_path


Back in ``move_towards()``, once the starting point is determined, one must then decide whether to go left or right around the circle. This calculation is done by the ``left_or_right()`` function::

        curr_ind = min_start_ind

        selector = left_or_right(curr_ind, min_end_ind, round_way_points)


        while curr_ind != min_end_ind:

            if check:  # Check if dropped
                rospy.loginfo("CHECKING IF DROPPED")
                if check_dropped():  # Exit and return failure
                    rospy.loginfo("DROPPED BRICK!")
                    return False

            # move arm to the curr node positon
            curr_node = round_way_points[curr_ind]
            print("MOVING ARM")
            print("CURR NODE Z: ", curr_node[0][2])


Finally the arm is moved to the neighbour way point in the circle until it reachs the way point with the exit id calculated at the begging of the function call. As you can see, the same ``move_arm()`` function
is used::


            move_arm([curr_node[0][0], curr_node[0][1], curr_node[0][2]+0.1, 3.14, 0, 3.14 / 4])
            curr_ind = curr_node[1][selector]  # go one way around the circle
        return True


.. _arm_master_main.py: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
.. _move_arm_server.py: https://github.com/de3-robo/arm_master/blob/master/scripts/move_arm_server.py
.. _brick_manager_server.py: https://github.com/de3-robo/arm_master/blob/master/scripts/brick_manager_server.py
.. _light.launch: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
.. _panda_one_brick.launch: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
.. _sim.launch: https://github.com/de3-robo/arm_master/blob/master/scripts/arm_master_main.py
