.. _physics:

Physics
==================

.. highlight:: python

Since SAPIEN is a physical simulation framework, we would like to showcase how to change physical properties which lead to different behaviors.

In this tutorial, you will learn the following:

* Use ``SceneConfig`` to initialize default physical properties
* Use ``PhysicalMaterial`` to set different physical materials
* Create kinematic actors
* Enable damping for actors
* Get kinematic quantities (pose, velocity, angular velocity) of an actor

The example illustrates an object sliding down the slope.
You can run the script with different arguments.
``transforms3d`` is required to compute poses, which can be installed by ``pip install transforms3d``.

.. figure:: assets/physics.gif
    :width: 640px
    :align: center
    :figclass: align-center

The full script can be downloaded here :download:`physics.py <../../../../examples/basic/physics.py>`

Set default physical properties
-------------------------------------

Default physical properties can be specified when a scene is created.
Those properties include gravity, static and dynamic friction, as well as `restitution <https://en.wikipedia.org/wiki/Coefficient_of_restitution>`_ (elasticity of collision).

.. literalinclude:: ../../../../examples/basic/physics.py
   :dedent: 0
   :lines: 99-114

``SceneConfig`` describes default physical properties, and can be passed to ``Scene``.

Set physical materials
-------------------------------------

``PhysicalMaterial`` describes physical (contact) properties (friction and restitution) of the material of an actor.
It can be specified when an actor is created.
If not provided, the default physical material, induced by the scene's default physical properties, will be used.
Note that ``PhysicalMaterial`` can only be created by ``create_physical_material(...)``.

.. literalinclude:: ../../../../examples/basic/physics.py
   :dedent: 0
   :lines: 119-123

Some other physical properties, like density, are directly provided to collision shapes. We update ``create_sphere`` function in :ref:`create_actors`.

.. literalinclude:: ../../../../examples/basic/physics.py
   :dedent: 0
   :lines: 60-75
   :emphasize-lines: 6,7,12

.. note::
   The rolling resistance (friction) is not modeled in SAPIEN currently.

Create a kinematic actor
-------------------------------------

Now, let's create a slope.
The slope should be a **kinematic** object, rather than a **dynamic** object.
In other words, it can not be affected by external forces.
We can set ``is_kinematic=True`` when building the actor.

.. literalinclude:: ../../../../examples/basic/physics.py
   :dedent: 0
   :lines: 23-57
   :emphasize-lines: 6,30,31

Set damping for the actor
-------------------------------------

Sometimes, you might model some resistance proportional to (linear or angular) velocity, like air resistance.
It can be achieved by setting the **damping** of an actor.

.. literalinclude:: ../../../../examples/basic/physics.py
   :dedent: 0
   :lines: 185

Get kinematic quantities (pose, velocity) of an actor
------------------------------------------------------------

We can acquire kinematic quantities (pose, linear velocity, angular velocity) of an actor through ``get_pose()``, ``get_velocity()``, ``get_angular_velocity()``.

.. literalinclude:: ../../../../examples/basic/physics.py
   :dedent: 0
   :lines: 210-212
