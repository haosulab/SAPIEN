.. _installation:

Installation
==================

.. highlight:: python

SAPIEN is distributed via `PyPI <https://pypi.org/project/sapien/>`_.

Currently supported Python versions:

* Python 3.7, 3.8, 3.9, 3.10, 3.11

Supported operating systems:

* Linux: Ubuntu 18.04+, Centos 7+, Arch

System requirements:

* Rendering: NVIDIA or AMD GPU
* Ray tracing: NVIDIA RTX GPU or AMD equivalent
* Denoising: NVIDIA RTX GPU

Software requirements:

* Ray tracing: NVIDIA Driver >= 470
* Denoising: NVIDIA Driver >= 522 (earlier version may work but is not officially supported)

Pip(PyPI) or Conda
-----------------------

.. code-block:: shell

  pip install sapien

.. note::
   ``pip >= 19.3`` is required for installation. Upgrade pip with

  .. code-block:: shell

     pip install -U pip

Build from source
-----------------------

You may build SAPIEN from source to access latest features under development in
the `dev <https://github.com/haosulab/SAPIEN/tree/dev>`_ branch, and/or
contribute to the project.

Clone SAPIEN
^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: shell

   git clone --recursive https://github.com/haosulab/SAPIEN.git

Build in Docker
^^^^^^^^^^^^^^^^^^^^^^

While it is possible to build SAPIEN natively on Linux. We strongly recommend
building using `Docker <https://docs.docker.com/get-started/overview/>`_.

.. code-block:: shell

   cd SAPIEN
   ./docker_build_wheels.sh

.. note::

   ``docker_build_wheels.sh`` builds for all Python versions by default. To
   build for specific versions, modify the last few lines of ``build_all.sh``.

.. note::

   Building may fail if you have previously built SAPIEN with Docker due to an
   update to the Docker image. Pull the latest Docker image with

  .. code-block:: shell

     docker pull fxiangucsd/sapien-build-env


Verify Installation
-----------------------

Server (no display)
^^^^^^^^^^^^^^^^^^^^^^^
.. warning::

   This script will generate ``output.png`` at the current working directory.

You may test the offscreen rendering of SAPIEN with the following command

.. code-block:: shell

   python -m sapien.example.offscreen

On a server without display. It may generate errors about the display. You can
ignore these warnings.

If SAPIEN is installed properly. The following image will be generated at the
current working directory, named ``output.png``.

.. figure:: assets/example.offscreen.png
    :width: 120px
    :align: center
    :figclass: align-center

Desktop (with display)
^^^^^^^^^^^^^^^^^^^^^^^

You may test the onscreen rendering of SAPIEN with the following command

.. code-block:: shell

   python -m sapien.example.hello_world

This command should open a viewer window showing a red cube on the ground.
You can learn more about this scene in :ref:`hello_world`.
