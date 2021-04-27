.. _contact:

Contact
==================

.. highlight:: python

Contact information is useful to check whether two rigid bodies collide or whether an object is grasped by a gripper.
The example shows how to check the contact between two actors (one box supported by another box).

In this tutorial, you will learn the following:

* Get contact information from ``Contact``

The full script is included as follows:

.. literalinclude:: ../../../../examples/basic/contact.py
    :linenos:

You can call ``get_contacts`` to fetch all contacts after the current simulation step.
It returns a list of ``Contact``.
``contact.actor0`` and ``contact.actor1`` refer to two actors involved in the contact.
``contact.points`` contains a list of ``ContactPoint``.

For each contact point, 

* ``impulse``: the impulse applied on the first actor.
* ``normal``: the direction of impulse.
* ``position``: the point of application in the world frame.
* ``seperation``: minimum distance between two shapes involved in the contact.

.. note::
   ``Contact`` in SAPIEN does not mean that two actors are contacting each other.
   It will be generated when the contact is about to start or end, and, of course, when the contact is happening.