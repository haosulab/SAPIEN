.. _create_actors:

Create Actors
==================

.. highlight:: python

SAPIEN simulates rigid body dynamics.
In SAPIEN, **actor** is an alias of rigid body.

In this tutorial, you will learn the following:

* Create ``Actor`` using primitives (box, sphere, capsule)
* Create ``Actor`` using mesh files
* Use ``Pose`` to set the pose of an actor

.. figure:: assets/create_actors.png
    :width: 640px
    :align: center
    :figclass: align-center

The full script can be downloaded here :download:`create_actors.py <../../../../examples/basic/create_actors.py>`

Create an actor by a single primitive
-------------------------------------------

The primitives supported by SAPIEN include box, sphere and capsule.
Here we show an example about how to create a box.
Examples to create a sphere and a capsule can be found in the code provided.

.. literalinclude:: ../../../../examples/basic/create_actors.py
   :dedent: 0
   :lines: 19-46

``Actor`` (or rigid body) is created through ``ActorBuilder`` in SAPIEN.
An actor consists of both collision shapes (used for physical simulation) and visual shapes (used for rendering).
You can call ``add_box_collision`` and ``add_box_visual`` to add collision and visual shapes of an box respectively.

.. note::
   Collision shapes do not necessarily correspond to visual shapes. 
   For example, you might have a simple collision shape for fast simulation, but a complicated visual shape for realistic rendering.

Then, you might create a box as follows:

.. literalinclude:: ../../../../examples/basic/create_actors.py
   :dedent: 0
   :lines: 127-133

The pose of the box in the world frame can be specified by ``Pose``.
``Pose`` describes a 6D pose, consisting of a 3-dim position vector ``p`` and a 4-dim quaternion ``q`` (to represent the rotation, in the wxyz convention).

Create an actor by multiple primitives
-------------------------------------------

Next, we show an example to create an actor (table) by multiple boxes (a tabletop with four legs).

.. literalinclude:: ../../../../examples/basic/create_actors.py
   :dedent: 0
   :lines: 82-112

We can call ``add_box_collision(pose=Pose(...), ...)`` to set the pose of a collision shape in **the actor frame**.
Similarly, we can call ``add_box_visual(pose=Pose(...), ...)`` for a visual shape.
Note that ``table.set_pose(pose)`` sets the pose of the actor in **the world frame**.

Create an actor by a mesh file
-------------------------------------------

Apart from primitives, actors can also be created from mesh files.

.. literalinclude:: ../../../../examples/basic/create_actors.py
   :dedent: 0
   :lines: 157-161

.. note::
   Any collision shape in SAPIEN is required to be convex.
   To this end, a mesh will be "cooked" into a convex mesh before being used in the simulation.
   The converted convex mesh is cached at the same directory of the original mesh file.
   Thus, if the mesh file is changed, please remove the cache.

Remove an actor
-------------------------------------------

After an actor is built with ``actor = builder.build()``, You can call
``scene.remove_actor(actor)`` to remove it. Using a removed actor will result in
undefined behavior (usually a crash).
