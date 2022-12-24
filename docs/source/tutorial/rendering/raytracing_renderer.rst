.. _raytracing_renderer:


Raytracing Renderer
==========================

.. highlight:: python

In this tutorial, you will learn the following:

* Use of raytracing-based backend *Kuafu*

The full script can be downloaded from :download:`kuafu.py <../../../../examples/rendering/kuafu.py>` and :download:`kuafu_mat.py <../../../../examples/rendering/kuafu_mat.py>`


Raytracing vs. Rasterization
------------------------------------

In the previous tutorials, we have learned how to set up a basic scene with SAPIEN and acquire rendering results with `VulkanRenderer`. The `VulkanRenderer` is a high-efficiency rasterization-based renderer, making it suitable for data-intensive tasks such as reinforcement learning.

However, though fast, the rasterization-based renderer is not physically-grounded, cannot render many real-world effects *e.g.* indirect lighting, realistic shadows, reflections and refractions, making the results overly *flat* and lack realism. On the other end, raytracing renderer simulates how rays interact with objects in a physically correct manner, and produces images that can be indistinguishable from those captured by a camera.

.. figure:: assets/rst_vs_rt.png
   :width: 540px
   :align: center

   From *A Shader-Based Ray Tracing Engine*, Park et al.

Raytracing with SAPIEN
------------------------------------

SAPIEN 2.0 ships with a raytracing renderer backend, named *Kuafu* (`KuafuRenderer`). Switching to Kuafu is easy, in the previous camera example, we set up the renderer with:

::

  renderer = sapien.VulkanRenderer()

To use Kuafu instead of the rasterization renderer, simply replace the line with:

::

  renderer = sapien.KuafuRenderer()

That's it! You can now rerun the script with raytracing renderer. The result would look like:

.. figure:: assets/kuafu_color.png
   :width: 540px
   :align: center


.. note::
    1. The Kuafu backend currently (2.0a) only supports `Color` downloading. Trying to download `Position` and other data from the camera will fail.
    2. The Kuafu backend does not feature full support for viewers. It is recommended to debug and build scene with `VulkanRenderer` and produce the final results with `KuafuRenderer`.

You may find that the result looks more realistic with the raytracing renderer. However the result contains noise due to under-sampling. To reduce the noise, one way is increasing the sample-per-pixel (spp) for the renderer. To achieve this, we can pass a `KuafuConfig` object when creating the renderer:

::

  render_config = sapien.KuafuConfig()
  renderer_config.spp = 256
  renderer = sapien.KuafuRenderer(render_config)

Increasing the spp will affect the rendering speed directly. A cheaper way to reduce the noise is using a denoiser. Kuafu features an OptiX denoiser by NVIDIA. To enable the denoiser, we can set the `use_denoiser` flag in the config:

::

  render_config = sapien.KuafuConfig()
  renderer_config.spp = 64
  renderer_config.use_denoiser = True
  renderer = sapien.KuafuRenderer(render_config)


.. note::
    You are required to have an NVIDIA GPU with driver version > 470 installed to use the denoiser.


Advanced Material Support in Kuafu
------------------------------------

To demonstrate the diverse material support in Kuafu. We will create a scene in SAPIEN and render with both `VulkanRenderer` and `KuafuRenderer`. First, let's setup the environment:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 11-54

We add a flag `use_kuafu` to allow easy switching between `VulkanRenderer` and `KuafuRenderer`. Next, let's build the scene. First, we create a rough bluish sphere:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 56-64

Next, we create a transparent sphere:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 66-76

Generally, setting a large `transmission` value will lead to a transparent material. Similarly, we can add a capsule and a box with advanced materials:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 78-96

Finally, let's load an external mesh and assign a highly metallic material to that object:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 98-107

After building the scene, we can get rendering results from the camera:

.. literalinclude:: ../../../../examples/rendering/kuafu_mat.py
   :dedent: 0
   :lines: 109-116

.. figure:: assets/mat_v.png
    :width: 720px
    :align: center

    Result with `VulkanRenderer`

.. figure:: assets/mat_k.png
    :width: 720px
    :align: center

    Result with `KuafuRenderer`

GPU-Accelerated Sensor Simulation with Simsense
-----------------------------------------------
This section is deprecated. Please refer to :ref:`depth_sensor` for a more comprehensive tutorial.