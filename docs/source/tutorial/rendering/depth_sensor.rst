.. _depth_sensor:


Realistic Depth Sensor Simulation
=================================

.. highlight:: python

In this tutorial, you will learn the following:

* Use of GPU-accelerated depth sensor simulator *SimSense*

The full script can be downloaded from :download:`kuafu_sensor_cuda.py <../../../../examples/rendering/kuafu_sensor_cuda.py>`


Depth Simulation with GPU
-------------------------
.. note::
    You are required to have an NVIDIA GPU to use SimSense.

Backed by realistic rendering, SAPIEN is able to reproduce the full sensor pipeline and simulate realistic sensor depth. This is powered by SimSense, a GPU-accelerated depth sensor simulation module designed to simulate real-world depth sensor in real time.

Let's see an example of simulating depth with SimSense. We can reuse the scene in the `Advanced Material Support in Kuafu` section of :ref:`raytracing_renderer`.

.. figure:: assets/mat_k.png
    :width: 720px
    :align: center

    Scene to simulate depth

Starting from our previously built environment and Kuafu renderer (You can find the building code in the full script), let's add a SimSense sensor to the scene:

::

  from sapien.sensor import ActiveLightSensorCUDA
  sensor = ActiveLightSensorCUDA('sensor', renderer, scene, sensor_type='d415')

The ``ActiveLightSensorCUDA`` class has built-in sensor types such as ``d415``. We can also manually set all the sensor parameters by passing them to the class. Please check the API doc for details. The ``sensor`` behaves just like a mounted camera. You can ``set_pose``, and ``take_picture`` on the sensor:

.. literalinclude:: ../../../../examples/rendering/kuafu_sensor_cuda.py
   :dedent: 0
   :lines: 94-99

After calling ``take_picture``, we can download RGB image, IR image, depth and point cloud from the sensor:

.. literalinclude:: ../../../../examples/rendering/kuafu_sensor_cuda.py
   :dedent: 0
   :lines: 101-115

.. figure:: assets/sensor_ir.png
    :width: 720px
    :align: center

    Simulated infra-red image from ``ActiveLightSensorCUDA``.

.. figure:: assets/sensor_depth_cuda.png
    :width: 720px
    :align: center

    Simulated depth image from ``ActiveLightSensorCUDA``.

Depth Simulation with CPU
-------------------------
.. note::
    If you are using CPU implementation of depth simulation, please notice that the CPU and GPU implementations won't have identical results even if you set all parameters to be the same. This is due to different implementation details.

While we strongly recommend usage of SimSense as it's faster, configures better and provides more realistic results, SAPIEN also provides a CPU alternative of ``ActiveLightSensorCUDA`` for users without available GPU environment. To use CPU implementation, simply change ``ActiveLightSensorCUDA`` in the previous section into ``ActiveLightSensor``, and all other functions should work the same.

Notice that ``ActiveLightSensor`` might have slightly different adjustable parameters compared to ``ActiveLightSensorCUDA``. Please check the API documentation for more info.