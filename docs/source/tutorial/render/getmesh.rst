.. _getmesh:

Get rendering mesh
=======================================

.. highlight:: python
   :linenothreshold: 5

.. note::
   If you haven't already done so, make sure you've completed the steps in :ref:`basic_index` before starting this tutorial. This tutorial will be based on :ref:`render`.

Get rendering mesh from visual bodies
--------------------------------------------------------------------------------

For any ``Actor`` in SAPIEN, call ``get_visual_bodies()`` to get all visual body
associated with it. For each visual body, call ``get_render_shapes`` to get all
shapes associated to this visual body. the local pose of the shape relative to
the ``Actor`` is in its ``pose`` variable, and it also has a ``mesh`` variable
providing vertices, triangle indices, normals, uvs, tangents and bitangents.

The following code snippet shows how to get all the meshes of an articulation
and display them with open3d.

.. literalinclude:: ../../../../example/render/shape.py
   :lineno-start: 32
   :lines: 32-52

.. note:: Getting rendering meshes from SAPIEN involves reading the meshes from
   GPU thus being very slow. The user is responsible for caching them for future
   use.
