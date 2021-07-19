 .. _collision_avoidance:

Collision Avoidance
====================


In :ref:`plan_a_path`, we talked about how to plan paths for the robot. However, in that tutorial, we didn't take the environment model into account. The robot will avoid self-collisions (i.e., collisions between the robot links), but may collide with the environment.

This tutorial will introduce two ways two ways to avoid collisions: add environment point clouds and attach a box. As shown in the right figure, the robot needs to move the red box to the place of the green box while avoiding collision with the blue box. The full script can be downloaded here :download:`demo.py <../../../../examples/motion_planning/collision_avoidance.py>`.

.. image:: assets/collision1.gif
   :width: 32%
.. image:: assets/collision2.gif
   :width: 32%
.. image:: assets/collision3.gif
   :width: 32%

The above figures show the benefit of collision avoidance:

- Left: w/o point cloud, w/o attach. The robot arm hits the blue box. 
- Middle: w/ point cloud, w/o attach. The red box hits the blue box.
- Right: w/ point cloud, w/ attach. There is no collision.

Add environment point clouds
--------------------------------------

One way to model the environment and avoid collision is through point clouds. The point cloud may come from the sensor observations or be sampled from the mesh surfaces. For example, we can add a point cloud for the blue box with ``planner.update_point_cloud()``:

.. literalinclude:: ../../../../examples/motion_planning/collision_avoidance.py
   :dedent: 0
   :lines: 127-133
   :emphasize-lines: 6

``planner.update_point_cloud()`` takes two arguments. The first one is a NumPy array of shape :math:`(n \times 3)`, which describes the coordinates of the points. **The coordinates should be represented in the frame of the robot arm's root link**. The second (optional) argument is ``resolution = 1e-3``, which describes the resolution of each point. 

After adding the point cloud, we can avoid collisions between the robot and the point cloud by setting ``use_point_cloud`` to be True. Both ``planner.plan()`` and ``planner.plan_screw()`` support this flag:

.. literalinclude:: ../../../../examples/motion_planning/collision_avoidance.py
   :dedent: 0
   :lines: 135-145
   :emphasize-lines: 3, 6

You don't need to provide the point cloud for each ``planner.plan()`` or ``planner.plan_screw()`` call. You can use  ``planner.update_point_cloud()`` to update the point cloud once it's changed.

.. note::
    Please remember to remove the points of the robot arm if the points come from the sensor observation. Otherwise, there will always be collisions, and the planner may fail to plan a valid path.

Attach a box
--------------------------------------
As shown in the above figure (middle one), after adding the point cloud of the blue box, the robot will not collide with it. However, the red box moves with the robot, and it may still collide with the blue box. To address this issue, we can attach a box to the robot, so that we can avoid the collision between the attached box and the environment point cloud:

.. literalinclude:: ../../../../examples/motion_planning/collision_avoidance.py
   :dedent: 0
   :lines: 166

``planner.update_attached_box()`` takes three arguments:

- ``size``: a list with three elements indicates the size of the attached box.
- ``pose``: a list with seven elements indicates the relative pose from the box to the attached link. The first three elements describe the position part, and the remaining four elements describe the quaternion (wxyz) for the rotation part.
- ``link_id = -1``: optional, an integer indicates the id of the link that the box is attached to. The link id is determined by the ``user_link_names`` (during Configuration), and starts from 0. The default value -1 indicates the ``move_group`` link.

After adding the attached box, we can avoid collisions between the attached box and the point cloud by setting both ``use_point_cloud`` and ``use_attach`` to be True. Both ``planner.plan()`` and ``planner.plan_screw()`` support the flags.

You can use  ``planner.update_attached_box()`` to update the box once it's changed.

As shown in the above figure (the right one), after adding the point cloud of the blue box and attaching the red box to the ``move_group`` link, there is no collision.