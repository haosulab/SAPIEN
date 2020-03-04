A minimal example
==================

.. highlight:: python
   :linenothreshold: 5

SAPIEN provides a serial of API to build and evaluate your simulation environment.
Here is a minimal example to create an simulation environment and visualize it.

Installation
------------

First install package dependencies.

.. code-block:: bash

    apt install libx11-6 libassimp4 libxcb1 libglew2.0

Then we install SAPIEN python package from pre-built package

.. code::

   pip install http://sapien.ucsd.edu/api/wheel/sapien-0.3.0.dev0-cp36-cp36m-linux_x86_64.whl

Simulation engine and scene
-------------------------------------

To use SAPIEN simulation, first create a simulation engine and use engine to build a simulation scene.

.. literalinclude:: ../../../../example/basic/minimal.py
   :lineno-start: 1
   :lines: 1-5

An ``engine`` in SAPIEN is the most basic interface for physical simulation, which can create a simulation scene.
``scene`` is a simulation instance where individual simulation runs on and can take ``step()``.
It is something like ``env`` in OpenAI Gym.
You can create multiple scene which can simulation step independently.

Renderer and renderer controller
------------------------------------

An ``renderer`` in SAPIEN is the most basic interface for rendering, which can create



