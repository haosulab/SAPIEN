.. _create_articulations:

Create Articulations
=====================

.. highlight:: python

The **articulation** is a set of **links** (each of which behaves like a rigid body) connected together with special **joints**.
For instance, a drawer can be connected to a table by a **prismatic** joint (slider), and a door can be connected to a frame by a **revolute** joint (hinge).
A robot is also an instance of an articulation.
Articulations are usually loaded from `URDF XML <http://wiki.ros.org/urdf/XML>`_, which we will see in other examples.
This tutorial showcases how to create an articulation programmatically by SAPIEN.

Articulations can be controlled by applying **torques** on their joints.
To drive an articulation to a desired state, users can apply a `controller <https://en.wikipedia.org/wiki/Control_theory>`_ to compute corrective torques according to the difference between the actual and the desired.

In this tutorial, you will learn the following:

* Create ``Articulation``
* Control the articulation with builtin controllers
* Get kinematic quantities of the articulation

The example illustrates how to build a controllable toy car from scratch.
``transforms3d`` is required.

.. figure:: assets/create_articulations.gif
    :width: 640px
    :align: center
    :figclass: align-center

The full script can be downloaded here :download:`create_articulations.py <../../../../examples/basic/create_articulations.py>`

Create a root link
-------------------------------------------

In SAPIEN, the articulation is represented as a tree.
Each node is a link, and each edge indicates that the child link is connected to the parent link by a joint.
To build a toy car, let's start with the car body.

.. literalinclude:: ../../../../examples/basic/create_articulations.py
   :dedent: 0
   :lines: 19-36
   :emphasize-lines: 12,15

``Articulation`` is created by ``ArticulationBuilder``.
Each link is built by ``LinkBuilder``, which can be created by an articulation builder.
A link is just a rigid body, and thus collision and visual shapes can be added.
A root link is created when ``create_link_builder()`` is called without specifying the parent link.

Create a child link connected by a revolute joint
----------------------------------------------------

Next, we create a child link (front steering shaft) connected to the root link (car body) by a revolute joint.

.. literalinclude:: ../../../../examples/basic/create_articulations.py
   :dedent: 0
   :lines: 38-60

A child link is created when ``create_link_builder(parent_link)`` is called with specifying the parent link.
Besides, we need to configure the joint.

There are multiple types of joints: prismatic, revolute, fixed.
The definitions follow `PhysX <https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/Joints.html>`_.

* **revolute**: a revolute joint (also called a hinge) keeps the origins and x-axes of the frames together, and allows free rotation around this common axis.
* **prismatic**: a prismatic joint (also called a slider) keeps the orientations identical, but allows the origin of each frame to slide freely along the common x-axis.
* **fixed**: a fixed joint locks the orientations and origins rigidly together

The location of the joint is defined by the joint pose in the parent frame ``pose_in_parent``, and the joint pose in the child frame ``pose_in_child``.

Other properties of a joint, like joint friction and joint damping, can also be set through ``set_joint_properties(...)``.

Control an articulation with builtin drives
----------------------------------------------------

After building the articulation, we want to control it by actuating its joints.
SAPIEN provides builtin **drives** (controllers) to control either the position or the speed of a joint.

.. literalinclude:: ../../../../examples/basic/create_articulations.py
   :dedent: 0
   :lines: 158-162

All the joints of an articulation can be acquired by ``get_joints()``.

.. note::
   Although the order of joints returned by ``get_joints()`` is fixed, it is recommended to index a joint by its name.
   Joint names should be unique, which is not forced in SAPIEN though. 

.. literalinclude:: ../../../../examples/basic/create_articulations.py
   :dedent: 0
   :lines: 218-222

For each active joint (with non-zero degree of freedom), we can set its drive properties: ``stiffness`` and ``damping``.
They implies the extent to which the drive attempts to achieve the target position and velocity respectively.
There do not exist a general rule to set those values and you usually should tune them case by case.
If you are familiar with control theory, they correspond to *P* and *D* terms in `PID controller <https://en.wikipedia.org/wiki/PID_controller>`_.
The initial target position and velocity of a joint are zero by default.
Since our toy car is designed to be a front-wheel drive car, we set both the stiffness and damping as zero for the back gear.

.. note::
   When a non-zero target is set and stiffness/damping is also non-zero, the drive takes effect internally at each simulation step.

We can implement different behaviors when different keys are pressed.
``set_drive_target(...)`` and ``set_drive_velocity_target(...)`` are called to set the target position and velocity of a joint drive.

.. literalinclude:: ../../../../examples/basic/create_articulations.py
   :dedent: 0
   :lines: 225-257

Get kinematic quantities of the articulation
------------------------------------------------------------

The pose of the articulation (frame) in the world frame can be acquired by ``get_pose()``.
It is the same as the pose of the root link.
Besides, joint positions and velocities can be acquired by ``get_qpos()`` and ``get_qvel()``.
They both return a list of scalars, the length of which is the total degree of freedom.
The order is the same as ``get_joints()``.

.. literalinclude:: ../../../../examples/basic/create_articulations.py
   :dedent: 0
   :lines: 266-268

Remove an articulation
-------------------------------------------

Similar to removing an actor, ``scene.remove_articulation(articulation)`` will
remove it from the scene. Using the articulation or any of its links or joints
after removal will result in undefined behavior (usually a crash).
