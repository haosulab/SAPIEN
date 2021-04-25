.. _shader:

Custom shaders
=======================================

.. highlight:: python
   :linenothreshold: 5

.. note::
   If you haven't already done so, make sure you've completed the steps in :ref:`basic_index` before starting this tutorial. This tutorial will be based on :ref:`render`.

Setup
--------------------------------------------------------------------------------

For this tutorial, we will start from the following code. Do remember replacing the token with your access token.

.. literalinclude:: ../../../../example/render/starter.py

Use custom shaders
--------------------------------------------------------------------------------

Vulkan renderer uses the spir-v shader format, which can be compiled from other
shader languages. Here we will introduce how to compile glsl shaders and use
them in SAPIEN. First download the original shader pack :download:`original.zip <../../../../example/original.zip>`
and unzip.

This renderer uses a deferred shading pipeline with forward transparency pass.
Files starting with ``gbuffer`` processes opaque geometry to create gbuffer
textures. Files starting with ``deferred`` computes lighting from the gbuffer
textures. Files starting with ``transparency`` computes another forward
transparency pass after deferred shading. Files starting with ``axis`` are for
axis drawing in the GUI. Files staring with ``composite`` are also for
displaying to the GUI window.

To compile glsl to spir-v, you will need a tool called `glslc <https://github.com/google/shaderc/tree/main/glslc>`_.
After installing glslc you just need to run ``glslc -c *`` in the shader folder containing glsl files to compile them into spir-v shader files. Now to use the shaders, you just need to set the shader path for the VulkanRenderer immediately after its creation.

.. literalinclude:: ../../../../example/render/world_normal.py
   :lineno-start: 13
   :lines: 13-14

Custom shader: world normal
--------------------------------------------------------------------------------

Now let's try to add the functionality of getting world normal instead of
camera space normal. You need to edit the file ``gbuffer.vert``. Change the line
``objectCoord = pos;`` as follows.

.. literalinclude:: ../../../../example/shader/gbuffer.vert
   :lineno-start: 37
   :lines: 37-38

You can see, previously, we are storing the local coordinates of the object into
the shader variable ``objectCoord``. This variable eventually shows up in the
custom texture. Now we just change it to show world normal.

Now in the SAPIEN controller GUI, if you switch to the "custom" display mode.
You can see world normals on the object. You can compare it with the
camera-space normal shown in the "normal" display mode. If you have a mounted
camera, you can get this texture by calling ``camera.get_custom_rgba()``.

Per-object data
--------------------------------------------------------------------------------

SAPIEN also supports passing per-object constant into the shader. This enables
per-object dynamic effects.

First, to pass in custom data, we need to call ``set_custom_data`` on an
``VisualBody``. For any ``Actor``, we call ``get_visual_bodies`` to get all
visual bodies associated with this actor. The function ``set_custom_data``
should always send in a list of 16 numbers representing a 4x4 matrix, but you do
not have to use it as a matrix.

For example, the following code sends in increasing values from 0 to 1 into
every visual body of the loaded articulation.

.. literalinclude:: ../../../../example/render/color.py
   :lineno-start: 34
   :lines: 34-46

Now let's adjust the shaders to display some light intensity changes. First we
make the following changes to the ``gbuffer.vert`` file.

.. literalinclude:: ../../../../example/shader2/gbuffer.vert
   :lineno-start: 28
   :lines: 28-29

.. literalinclude:: ../../../../example/shader2/gbuffer.vert
   :lineno-start: 38
   :lines: 38-39

Here we pass a intensity value from the per-object user data into the fragment
shader. Now in fragment shader ``gbuffer.frag``, we modify the albedo
computation by

.. literalinclude:: ../../../../example/shader2/gbuffer.frag
   :lineno-start: 38
   :lines: 38-42

Now you should see a chair with changing light intensity.
