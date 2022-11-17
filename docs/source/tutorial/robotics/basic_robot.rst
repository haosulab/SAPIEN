.. _basic_robot:

Getting Started with Robot
===========================

.. highlight:: python

.. note::
   Please first complete :ref:`basic_index` before continuing this tutorial.
   The assets (robot) used in this tutorial can be found `here <https://github.com/haosulab/SAPIEN-Release/tree/master/examples/assets>`__.

In this tutorial, you will learn the following:

* Load a robot (URDF)
* Set joint positions
* Compensate passive forces
* Control the robot by torques

The full script can be downloaded here :download:`basic_robot.py <../../../../examples/robotics/basic_robot.py>`

Set up the engine, renderer and scene
-----------------------------------------

First of all, let's set up the simulation environment as illustrated in :ref:`hello_world`.

.. literalinclude:: ../../../../examples/robotics/basic_robot.py
   :dedent: 0
   :lines: 5-22

Load a robot URDF
-----------------------------------------

Now, you can create a ``URDFLoader`` to load the URDF XML of Kinova Jaco2 arm.
`URDF XML <http://wiki.ros.org/urdf/XML>`_ describes a robot.
Usually, URDF files are provided by manufacturers.
For example, the URDF XML of Kinova Jaco2 arm can be found `here <https://github.com/Kinovarobotics/kinova-ros>`__.

.. literalinclude:: ../../../../examples/robotics/basic_robot.py
   :dedent: 0
   :lines: 25-28

Note that there is a ``fix_root_link`` flag for the URDF loader.
If it is true (by default), then the root link of the robot will be fixed.
Otherwise, it is allowed to move freely.

The robot is loaded as ``Articulation``, which is a tree of links connected by joints.
We can set the pose of its root link through ``set_root_pose(...)``.

If you run the example with ``demo(fix_root_link=False, balance_passive_force=False)``, it is expected that you will observe the following "falling-down" robot arm.
We will see how to keep the robot at a certain pose later.

.. figure:: assets/robot_fall.gif
    :width: 640px
    :align: center
    :figclass: align-center

    The robot arm falls down.

.. note::
   When a robot is already loaded, changing the flag of the URDF loader will not take effect.

Set joint positions
--------------------------------------

.. literalinclude:: ../../../../examples/robotics/basic_robot.py
   :dedent: 0
   :lines: 31-34

We can also set initial joint positions through ``set_qpos(qpos=...)``.
The ``qpos`` should be a concatenation of the position of each joint.
Its length is the degree of freedom, and its order is the same as that returned by ``robot.get_joints()``.

.. note::
   If the articulation is loaded from a URDF file, its joints are in preorder (DFS preorder traversal over the articulation tree).
   If the articulation is built programmatically (refer to :ref:`create_articulations`), its joints are in the order when they are built.

Compensate passive forces (e.g. gravity)
-----------------------------------------

You may find that even if you run the example with ``fix_root_link=True``, the robot still can not maintain its initial joint positions.
It is due to gravitational force and other possible passive forces, like Coriolis and Centrifugal force.

.. figure:: assets/robot_fix.gif
    :width: 640px
    :align: center
    :figclass: align-center

    The root link (base) of the robot is fixed, but it still falls down due to passive forces.

For a real robot, gravity compensation is done by an internal controller hardware.
So it is usually desirable to skip this troublesome calculation of how to compensate gravity.
SAPIEN provides ``compute_passive_force`` to compute desired forces or torques on joints to compensate passive forces.
In this example, we only consider gravity as well as coriolis and centrifugal force.

.. literalinclude:: ../../../../examples/robotics/basic_robot.py
   :dedent: 0
   :lines: 36-46

We recompute the compensative torque every step and control the robot by ``set_qf(qf)``.
``qf`` should be a concatenation of the force or torque to apply on each joint.
Its length is the degree of freedom, and its order is the same as that returned by ``robot.get_joints()``.
Note that when ``qf`` is set, it will be applied every simulation step.
You can call ``robot.get_qf()`` to acquire its current value.

Now, if you run the example with ``demo(fix_root_link=True, balance_passive_force=True)``, it is observed that the robot can stay at the target pose for a short period.
However, it will then deviate from this pose gradually due to numerical error.

.. figure:: assets/robot_fix_balance.gif
    :width: 640px
    :align: center
    :figclass: align-center

    The robot arm is able to stay at the target pose, but might deviate gradually due to numerical error.
    The animation is accelerated.

.. note::
   To avoid deviating from the target pose gradually,
   either we specify the damping (resistence proportional to velocity) of each joint in the URDF XML, or a controller can be used to compute desired extra forces or torques to keep the robot around the target pose.
   :ref:`pid` will elaborate how to control the robot with a controller.
