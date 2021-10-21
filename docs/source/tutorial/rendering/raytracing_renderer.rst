.. _raytracing_renderer:


Raytracing Renderer
==================

.. highlight:: python

In this tutorial, you will learn the following:

* Use of raytracing-based backend *Kuafu*

The full script can be downloaded from :download:`kuafu.py <../../../../examples/rendering/kuafu.py>`


Raytracing vs. rasterization
------------------

In the previous tutorials, we have learned how to set up a basic scene with SAPIEN and acquire rendering results with `VulkanRenderer`. The `VulkanRenderer` is a high-efficiency rasterization-based renderer, making it suitable for data-intensive tasks such as reinforcement learning.

However, though fast, the rasterization-based renderer is not physically-grounded, cannot render many real-world effects *e.g.* indirect lighting, realistic shadows, reflections and refractions, making the results overly *flat* and lack realism. On the other end, raytracing renderer simulates how rays interact with objects in a physically correct manner, and produces images that can be indistinguishable from those captured by a camera.

TODO: Figures.

SAPIEN 2.0 ships with a raytracing renderer backend, named *Kuafu* (`KuafuRenderer`). Switching to Kuafu is easy, in the precious camera example, we set up the renderer with:

::

  renderer = sapien.VulkanRenderer()

To use Kuafu instead of the rasterization renderer, simply replace that with:

::

  renderer = sapien.KuafuRenderer()

That's it! You can now rerun the script with raytracing renderer.








