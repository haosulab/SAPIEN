from __future__ import annotations
import numpy
import sapien.pysapien
import typing
__all__ = ['pose_gl_to_ros', 'pose_ros_to_gl', 'shortest_rotation']
def shortest_rotation(source: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]], target: numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]) -> numpy.ndarray[typing.Literal[4], numpy.dtype[numpy.float32]]:
    ...
pose_gl_to_ros: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, -0.5, 0.5, 0.5])
pose_ros_to_gl: sapien.pysapien.Pose  # value = Pose([0, 0, 0], [-0.5, 0.5, -0.5, -0.5])
