import numpy as np
from transforms3d.quaternions import axangle2quat as aa
from transforms3d.quaternions import qmult, rotate_vector
from sapien import Pose


class FPSCameraController:
    def __init__(self):
        self.forward = np.array([1, 0, 0])
        self.up = np.array([0, 0, 1])
        self.left = np.cross(self.up, self.forward)
        self.xyz = np.zeros(3)
        self.rpy = np.zeros(3)
        self.update()

    def setRPY(self, roll, pitch, yaw):
        self.rpy = np.array([roll, pitch, yaw])
        self.update()

    def setXYZ(self, x, y, z):
        self.xyz = np.array([x, y, z])
        self.update()

    def move(self, forward, left, up):
        q = qmult(
            qmult(aa(self.up, -self.rpy[2]), aa(self.left, -self.rpy[1])),
            aa(self.forward, self.rpy[0]),
        )
        self.xyz = self.xyz + (
            rotate_vector(self.forward, q) * forward
            + rotate_vector(self.left, q) * left
            + rotate_vector(self.up, q) * up
        )
        self.update()

    def rotate(self, roll, pitch, yaw):
        self.rpy = self.rpy + np.array([roll, pitch, yaw])
        self.update()

    def update(self):
        self.rpy[1] = np.clip(self.rpy[1], -1.57, 1.57)
        if self.rpy[2] >= 3.15:
            self.rpy[2] = self.rpy[2] - 2 * np.pi
        elif self.rpy[2] <= -3.15:
            self.rpy[2] = self.rpy[2] + 2 * np.pi
        rot = qmult(
            qmult(aa(self.up, -self.rpy[2]), aa(self.left, -self.rpy[1])),
            aa(self.forward, self.rpy[0]),
        )
        self._pose = Pose(self.xyz, rot)

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose: Pose):
        # TODO: finish this
        pass


class ArcRotateCameraController:
    def __init__(self):
        self.forward = np.array([1, 0, 0])
        self.up = np.array([0, 0, 1])
        self.left = np.cross(self.up, self.forward)
        self.center = np.zeros(3)
        self.yaw = 0
        self.pitch = 0
        self.radius = 1

        self.update()

    def set_center(self, center):
        self.center = np.array(center)
        self.update()

    def rotate_yaw_pitch(self, yaw, pitch):
        self.yaw += yaw
        self.pitch += pitch
        self.update()

    def set_yaw_pitch(self, yaw, pitch):
        self.yaw = yaw
        self.pitch = pitch
        self.update()

    def zoom(self, zoom_in):
        self.radius -= zoom_in
        self.radius = max(0.1, self.radius)
        self.radius = min(100, self.radius)
        self.update()

    def set_zoom(self, zoom):
        self.radius = zoom
        self.update()

    def update(self):
        rot = qmult(aa(self.up, self.yaw), aa(self.left, self.pitch))
        pos = self.center - self.radius * rotate_vector(np.array([1, 0, 0]), rot)
        self._pose = Pose(pos, rot)

    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose: Pose):
        # TODO: finish this
        pass
