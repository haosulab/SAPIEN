.. _raytracing_renderer:


Ray tracing Renderer
==========================

.. highlight:: python

In this tutorial, you will learn the following:

* Use ray tracing in ``SapienRenderer``
* Use GPU-accelerated depth computing module *SimSense*

The full script can be downloaded from :download:`rt.py <../../../../examples/rendering/rt.py>`, :download:`rt_mat.py <../../../../examples/rendering/rt_mat.py>`.

Ray tracing vs. rasterization
------------------------------------

In the previous tutorials, we have learned how to set up a basic scene with
SAPIEN and acquire rendering results under the default settings using
`SapienRenderer`. By default, `SapienRenderer` uses a high-efficiency
rasterization-based rendering pipeline, making it suitable for data-intensive
tasks such as reinforcement learning.

However, though fast, the rasterization-based renderer is not
physically-grounded, and cannot faithfully model many real-world effects,
*e.g.*, indirect lighting, realistic shadows, reflections and refractions,
making the results overly flat and lack realism. On the other end, ray tracing
renderer simulates how light rays interact with objects in a physically correct
manner, and produces images that can be indistinguishable from those captured by
a camera.

.. figure:: assets/rst_vs_rt.png
   :width: 540px
   :align: center

   From *A Shader-Based Ray Tracing Engine*, Park et al.

Ray tracing with SAPIEN
------------------------------------

In SAPIEN 2.2, the default renderer ``SapienRenderer`` (formerly known as
``VulkanRenderer``) supports both rasterization and ray tracing, and different
cameras can use differet rendering pipelines. Choosing a pipeline is done
through specifying a `shader pack`, which is a directory containing a collection
of `glsl` files.

To use the ray-tracing pipeline, simply add the following lines before creating a
camera or a viewer.

.. literalinclude:: ../../../../examples/rendering/rt.py
   :dedent: 0
   :lines: 17-18

That's it! You can now rerun the script with raytracing renderer. The result would look like:

.. figure:: assets/kuafu_color.png
   :width: 540px
   :align: center


You may find that the result looks more realistic with the ray tracing shader.
However the result contains noise due to under-sampling. To reduce the noise,
one way is to increase the sample-per-pixel for the renderer. To achieve this,
simply change the ``rt_samples_per_pixel`` in ``render_config``.

.. literalinclude:: ../../../../examples/rendering/rt.py
   :dedent: 0
   :lines: 19

Increasing the spp will affect the rendering speed directly. A cheaper way to reduce the noise is using a denoiser. ``SapienRenderer`` supports the OptiX denoiser on NVIDIA RTX GPUs.

.. literalinclude:: ../../../../examples/rendering/rt.py
   :dedent: 0
   :lines: 20

.. note::
   You are required to have an NVIDIA RTX GPU with driver version >= 522 installed to use the denoiser.

   While you may get the denoiser to work on drivers of lower versions (it has
   worked on 470 in one of our tests), it is not officially supported.

Reflection and refraction
------------------------------------

Ray tracing allows SAPIEN to render realistic reflection and refractions.

We will create a scene in SAPIEN and render with ray tracing turned on and off.
First, let's setup the environment:

.. literalinclude:: ../../../../examples/rendering/rt_mat.py
   :dedent: 0
   :lines: 11-51

We add a flag ``ray_tracing`` to allow switching between rasterization and ray
tracing. Next, let's build the scene. First, we create a rough bluish sphere:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 53-61

Next, we create a rough transparent sphere:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 63-73

Generally, setting a large `transmission` value will lead to a transparent
material. Similarly, we can add a capsule and a box with complex materials:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 75-93

Finally, let's load an external mesh and assign a highly metallic material to that object:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 95-104

After building the scene, we can get rendering results from the camera:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 106-113

.. figure:: assets/mat_v.png
    :width: 720px
    :align: center

    Result with `VulkanRenderer`

.. figure:: assets/mat_k.png
    :width: 720px
    :align: center

    Result with `KuafuRenderer`


GPU-accelerated Sensor Simulation with SimSense
------------------------------------------------------------------------
.. note::
    You are required to have an NVIDIA GPU to use SimSense.

`ActiveLightSensor.get_depth()` is a function run on CPU and is rather slow (can take more than 1s for every function call). To address this, SAPIEN is incorporated with *SimSense*, a GPU-accelerated depth computing module powered with CUDA. SimSense can directly take the infra-red images rendered by Kuafu and compute depth map in a physically-grounded fashion. SimSense is able to accelerate the original <1 fps process to over 100 fps.

To use SimSense, all you have to do is changing the `ActiveLightSensor` class introduced in the previous section into `ActiveLightSensorCUDA`, and that's it! More specifically, what you need is:

::

  from sapien.sensor import ActiveLightSensorCUDA
  sensor = ActiveLightSensorCUDA('sensor', renderer, scene, sensor_type='d415')

After that, you can use every functions of `sensor` we've introduced in the same way as before, and SimSense will be automatically triggered when gets to its part.

With the goal of closing sim2real gap, SimSense is built to provide results that are close to modern real-world depth sensors. For the same example in the previous section, depth generated by SimSense is:

.. figure:: assets/sensor_depth_cuda.png
    :width: 720px
    :align: center

    Simulated depth image from `ActiveLightSensorCUDA`.

SimSense also provides a series of configurable parameters to adapt to different real-world sensors. For more information about the configurable parameters, please check the API document.
