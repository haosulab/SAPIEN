.. _migration_index:

Migration from sapien 1.x.x to 2.x.x
===================================================================

Light
-------------------------------------------------------------------

``renderer_scene`` variable in ``Scene`` class is no longer used. All ``renderer_scene.add_xxx_light`` functions have been moved the ``Scene`` class directly. For example, ``scene.renderer_scene.add_point_light`` should now be replaced by ``scene.add_point_light``. The return value is now inherited from the ``Entity`` class, which is similar to ``Actor`` and allows setting poses and names directly.

Camera
-------------------------------------------------------------------

Cameras in SAPIEN no longer requires mounts. In fact, they now also inherit from
the ``Entity`` class. Cameras can be directly added to the scene by
``Scene.add_camera``. Cameras can mount actors after they are added by calling
``Camera.set_parent`` and ``Camera.set_local_pose``. When the camera is not
mounted, ``Camera.set_local_pose`` sets its global pose.

Now, ``fovx`` parameter has been removed from ``Scene.add_mounted_camera``.
However, code that provides ``fovx`` will still work with a deprecation warning.

Functions related to `Mounted Camera` are removed or replaced.
``Scene.remove_mounted_camera`` has been replaced by ``Scene.remove_camera``.
``Scene.find_camera_by_mount`` has been removed.

Cameras now support full camera parameters through ``camera.near``,
``camera.far``, ``camera.set_fovx``, ``camera.set_fovy``,
``camera.set_focal_lengths``, ``camera.set_principal_point``, ``camera.skew``,
and the all-in-one method ``camera.set_perspective_parameters``.
