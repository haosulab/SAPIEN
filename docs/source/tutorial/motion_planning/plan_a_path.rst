 .. _plan_a_path:

Plan a Path
==================

.. highlight:: python

In this tutorial, we will talk about how to plan paths for the agent. As shown in the demo, the robot needs to move the three boxes a bit forward. The full script can be downloaded here :download:`demo.py <../../../../examples/motion_planning/demo.py>`.

.. figure:: assets/RRT.gif
    :width: 320px
    :align: left
    :figclass: align-left

    plan with RRTConnect

.. figure:: assets/screw.gif
    :width: 320px
    :align: right
    :figclass: align-right

    plan with screw motion

.. note::
    This tutorial only talks about the basic usages, and the robot only avoids self-collisions (i.e., collisions between the robot links) in this demo. Please refer to :ref:`collision_avoidance` to include the environment model and other advanced usages. 

Plan with sampling-based algorithms
--------------------------------------

``mplib`` supports state-of-the-art sampling-based motion planning algorithms by leveraging `OMPL <https://github.com/ompl/ompl>`_. You can call ``planner.plan()`` to plan a path for moving the ``move_group`` link to a target pose: 

.. literalinclude:: ../../../../examples/motion_planning/demo.py
   :dedent: 0
   :lines: 127-133
   :emphasize-lines: 2

Specifically, ``planner.plan()`` takes two required arguments as input. The first one is the target pose of the ``move_group`` link. It's a 7-dim list, where the first three elements describe the position part, and the remaining four elements describe the quaternion (wxyz) for the rotation part. **Note that the pose is relative to the frame of the robot's root link**. The second argument is the current joint positions of all the active joints (e.g., given by SAPIEN). The ``planner.plan()`` function first solves the inverse kinematics to get the joint positions for the target pose. It then calls the RRTConnect algorithm to find a path in the joint space. Finally, it parameterizes the path to generate time, velocity, and acceleration information.

``planner.plan()`` returns a dict which includes:   

- ``status``: a string indicates the status:

    - ``Success``: planned a path successfully.
    - ``IK Failed``: failed to solve the inverse kinematics. This may happen when the target pose is not reachable.
    - ``RRT Failed``: failed to find a valid path in the joint space. This may happen when there is no valid path or the task is too complicated.
- ``position``: a NumPy array of shape :math:`(n \times m)` describes the joint positions of the waypoints. :math:`n` is the number of waypoints in the path, and each row describes a waypoint. :math:`m` is the number of active joints that affect the pose of the ``move_group`` link. For example, for our panda robot arm, each row includes the positions for the first seven joints. 
- ``duration``: a scalar indicates the duration of the output path. ``mplib`` returns the optimal duration considering the velocity and acceleration constraints. 
- ``time``: a NumPy array of shape :math:`(n)` describes the time step of each waypoint. The first element is equal to 0, and the last one is equal to the ``duration``. Argument ``time_step`` determines the interval of the elements.
- ``velocity``: a NumPy array of shape :math:`(n \times m)` describes the joint velocities of the waypoints. 
- ``acceleration``: a NumPy array of shape :math:`(n \times m)` describing the joint accelerations of the waypoints. 


``planner.plan()`` also takes other optional arguments with default values:

- ``time_step = 0.1``: ``time_step`` specify the time interval between the waypoints. The larger the value, the sparser the output waypoints. In this demo, we align the ``time_step`` with SAPIEN's time step.
- ``rrt_range = 0.1``: the incremental distance in the RRTConnect algorithm.  The larger the value, the sparser the sampled waypoints (before time parameterization).
- ``planning_time=1``: time limit for RRTConnect algorithm, in seconds.
- ``fix_joint_limits=True``: whether to clip the current joint positions if they are out of the joint limits.
- ``verbose=False``: whether to display some internal outputs.
- ``use_point_cloud=False`` and ``use_attach=False``: related to collision avoidance, will be discussed in :ref:`collision_avoidance`.



Follow a path
--------------------------------------
``plan()`` outputs a time-parameterized path, and we need to drive the robot to follow the path. See :ref:`pid` for some basic usages. Depending on your controller, you may only use the returned position information, or use the velocity and acceleration information as well.

In this demo, we use the PhysX internal PD controller. We first need to set the drive properties of the active joints at the very beginning:

.. literalinclude:: ../../../../examples/motion_planning/demo.py
   :dedent: 0
   :lines: 43-44

To follow a path, at each time step, we set the target position and target velocity according to the returned path. Please note that since we aligned the time step of the returned path with the SAPIEN time step, we don’t need to interpolate the returned path.

.. literalinclude:: ../../../../examples/motion_planning/demo.py
   :dedent: 0
   :lines: 86-99
   :emphasize-lines: 8-10

We also compensate the passive forces through ``set_qf()`` (see :ref:`basic_robot` for details).

You can also use your own controller.

.. note::
    If you find your robot doesn't move as expected, please **double-check** your controller, especially the controller's parameters. In many cases, the planner finds a good path while the controller fails to follow the path.


Plan with screw motion
--------------------------------------
Besides using the sampling-based algorithms, we also provide another simple way (trick) to plan a path. For some tasks, we can directly move the ``move_group`` link towards the target pose. It's internally achieved by first calculating the relative transformation from its current pose to the target pose, then calculating the relative transformation's exponential coordinates, and finally calculating the joint velocities with the Jacobian matrix.

Compared to the sampling-based algorithms, planning with screw motion has the following pros:

- faster: since it doesn't need to sample lots of states in the joint space, planning with screw motion can save lots of planning time.
- `straighter` path: there is no guarantee for sampling-based algorithms to generate `straight` paths even it's a simple lifting task since it connects states in the joint space. In contrast, the returned path by the exponential coordinates and the Jacobian matrix can sometimes be more reasonable. See the above figures for comparison. 


You can call ``planner.plan_screw()`` to plan a path with screw motion. Similar to ``planner.plan()``, it also takes two required arguments: target pose and current joint positions, and returns a dict containing the same set of elements. 

.. literalinclude:: ../../../../examples/motion_planning/demo.py
   :dedent: 0
   :lines: 135-143
   :emphasize-lines: 2

However, planning with screw motion only succeeds when there is no collision during the planning since it can not detour or replan. We thus recommend use ``planner.plan_screw()`` for some simple tasks or combined with ``planner.plan()``. As shown in the code, we first try ``planner.plan_screw()``, if it fails (e.g., collision during the planning), we then turn to the sampling-based algorithms. Other arguments are the same with ``planner.plan()``.

``planner.plan_screw()`` also takes ``qpos_step = 0.1``, ``time_step = 0.1``, ``use_point_cloud = False``, ``use_attach = False``, and ``verbose = False`` as optional arguments, where ``qpos_step`` specifies the incremental distance of the joint positions during the path generation (before time paramtertization).


Move the boxes
--------------------------------------

In this example, we manually mark some landmark poses to move the boxes:

.. literalinclude:: ../../../../examples/motion_planning/demo.py
   :dedent: 0
   :lines: 151-172

To control the gripper, we use ``set_drive_target()`` to set target positions for the two gripper joints:

.. literalinclude:: ../../../../examples/motion_planning/demo.py
   :dedent: 0
   :lines: 101-125
   :emphasize-lines: 3, 16