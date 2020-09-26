Use PID to drive the robot
==========================

.. note::
   If you haven’t already done so, make sure you’ve completed the steps in :doc:`get_start` before this tutorial.

.. highlight:: python
   :linenothreshold: 5


Basic setup
-----------

This part is similar as before with only few parameter change. In this tutorial, we will show how to drive
your robot to the target position(same as previous tutorial) use Physx's internal PD
or write a PID by yourself.

.. literalinclude:: ../../../../example/robotics/pid.py
   :lineno-start: 1
   :lines: 1-2, 38-63

Drive robot with Physx's internal PD controller
------------------------------------------------

To use Physx's internal drive, you can use the ``joint.set_drive_property`` function for each joint.
The stiffness and damping can be regarded as ``P`` and ``D`` in a typical PID controller.
Then, you can use ``set_drive_target`` to set the target position of either a single joint or a whole robot.
Then the internal solver in Physx will calculate the joint force in order to reach this position.

.. note::
   If you do not balance the passive force, e.g. gravity, you can never reach the desired pose given in ``set_drive_target`` due to steady-state-error.

.. literalinclude:: ../../../../example/robotics/pid.py
   :lineno-start: 69
   :lines: 69-72, 75
   :dedent: 4


Write your own PID controller
-----------------------------

If you really need the ``integrator I`` to implement a PID controller, you can write it by your own.
There is a very simple PID controller as follow, where parameters are not fine-tuned. You can add more
tricks when doing integration or propagate the error to make your controller more stable.
In most cases, it is always recommended to use the ``set_drive_target`` instead of your own PID. The Physx function is much more efficient and robustness when the parameters are not well-tuned.


.. literalinclude:: ../../../../example/robotics/pid.py
   :lineno-start: 5
   :lines: 5-28
   :dedent: 0

.. literalinclude:: ../../../../example/robotics/pid.py
   :lineno-start: 65
   :lines: 65-74
   :dedent: 4

.. literalinclude:: ../../../../example/robotics/pid.py
   :lineno-start: 79
   :lines: 79-88
   :dedent: 4

.. note::
   In practice, not use your own PID unless you really need to balance some steady-state-error which can not be compensated by ``compensate_passive_force``.


The entire code
------------------

The entire code is as follow:

.. literalinclude:: ../../../../example/robotics/pid.py


