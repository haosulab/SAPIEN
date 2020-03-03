Getting Started
================

.. note::
	If you haven’t already done so, make sure you’ve completed the steps in `it <>` before starting tutorial of the robotics part.

.. highlight:: python
   :linenothreshold: 5

Set up engine, renderer and scene
--------------------------------------

First setup the simulation engine, create a
simulation scene and visualize it using renderer controller like before

.. code::

	import pysapien as sapien


	sim = sapien.Engine()
	renderer = sapien.OptifuserRenderer()
	sim.set_renderer(renderer)
	renderer_controller = sapien.OptifuserController(renderer)
	renderer_controller.set_camera_position(-2, 0, 1)
	renderer_controller.set_camera_rotation(0, -0.5)

	scene0 = sim.create_scene(gravity=[0, 0, -9.81])
	renderer_controller.set_current_scene(scene0)
	scene0.add_ground(altitude=0)
	scene0.set_timestep(1 / 240)
	scene0.set_ambient_light([0.5, 0.5, 0.5])
	scene0.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])

Load robot and set joint position
--------------------------------------

Then we create a URDF loader to load the Kinova Jaco2 arm from URDF file
and set the robot to the initial position. Note that this is a
``fix_root_link`` flag for URDF loader. If it is set to true, then the robot root
will be fixed to world coordinates, otherwise the root link will be free with all 6
degree of freedom.

.. code:: python

	fix_robot_root = False
	loader = scene0.create_urdf_loader()
	loader.fix_root_link = fix_robot_root
	robot: sapien.Articulation = loader.load("assets/robot/jaco2.urdf") # TODO
	arm_init_qpos = [4.71, 2.84, 0, 0.75, 4.62, 4.48, 4.88]
	gripper_init_qpos = [0, 0, 0, 0, 0, 0]
	init_qpos = arm_init_qpos + gripper_init_qpos
	robot.set_qpos(init_qpos)

Like before, run simulation step and on-screen rendering.

.. code:: python

	steps = 0
	renderer_controller.show_window()
	while not renderer_controller.should_quit:
		scene0.update_render()
		for i in range(4):
			scene0.step()
			steps += 1
		renderer_controller.render()

Change flag of URDF Loader
--------------------------------------

Now we run all the code, it is expected that you will observe the following "falling-down"
robot arm. This is caused by the ``fix_root_link`` flag, which is set to ``False``
before load the robot.

.. figure:: assets/robot_fall.png
    :width: 640px
    :align: center
    :figclass: align-center

Change the flag to ``True`` and rerun the code, we still find that the robot arm can not
get to the initial position as we set. Actually, at the first simulation step, the robot
is set to our desired joint angle. However, due to gravitational force and other possible
passive force like Coriolis and Centrifugal force, the Jaco2 arm will immediately drop down.

.. note::
	When a robot is already loaded, change flags of URDF loader will have no effect on it.

Gravity compensation
--------------------------------------

For a real robot, the gravity compensation is done by a internal controller hardware.
So it is usually desirable to skip this troublesome calculation of how to compensate gravity.

In SAPIEN, we use ``compute_passive_force`` to do that. It calculates the joint force/torque
in order to compensate specified passive force. Here we only consider gravity, coriolis and centrifugal
force.

.. code-block:: python
	 :emphasize-lines: 9, 10
	 :linenos:
	 :dedent: 0

	 loader.fix_root_link = True
	 # code like before

	 steps = 0
	 renderer_controller.show_window()
	 while not renderer_controller.should_quit:
		 scene0.update_render()
		 for i in range(4):
			 qf = robot.compute_passive_force(gravity=True, coriolisAndCentrifugal=True, external=False)
			 robot.set_qf(qf)
			 scene0.step()
			 steps += 1
			 renderer_controller.render()

After running, we can see the robot can stay at the target pose for a short period as following.
However, it will then deviate from this pose gradually due to numerical error.

.. figure:: assets/robot_right_position.png
    :width: 640px
    :align: center
    :figclass: align-center

Add damping to joints
--------------------------------------

To make the robot stay at this pose, we need to add some dumping to the robot joint.
It will penalize the joint velocity to stabilize the robot. Add the following code
before the while loop.

.. code::

	for joint in robot.get_joints():
		joint.set_drive_property(stiffness=0, damping=10)

After that, our robot can keep it pose at the given joint angle. The entire code is as follow:

.. literalinclude:: ../../../../example/robotics/get_start.py

