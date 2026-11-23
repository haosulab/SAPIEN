from __future__ import annotations
import numpy
import numpy.typing
import sapien.pysapien
import typing
__all__ = ['PinocchioModel']
class PinocchioModel:
    @staticmethod
    def _pybind11_conduit_v1_(*args, **kwargs):
        ...
    def __init__(self, arg0: str, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"]) -> None:
        ...
    def compute_coriolis_matrix(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], qvel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        ...
    def compute_forward_dynamics(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], qvel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], qf: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
    def compute_forward_kinematics(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> None:
        """
        Compute and cache forward kinematics. After computation, use get_link_pose to retrieve the computed pose for a specific link.
        """
    def compute_full_jacobian(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> None:
        """
        Compute and cache Jacobian for all links
        """
    def compute_generalized_mass_matrix(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, n]"]:
        ...
    def compute_inverse_dynamics(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], qvel: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], qacc: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
    def compute_inverse_kinematics(self, link_index: int, pose: sapien.pysapien.Pose, initial_qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"] = ..., active_qmask: typing.Annotated[numpy.typing.ArrayLike, numpy.int32, "[m, 1]"] = ..., eps: float = 0.0001, max_iterations: int = 1000, dt: float = 0.1, damp: float = 1e-06) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"], bool, typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, 1]"]]:
        """
        Compute inverse kinematics with CLIK algorithm.
        Details see https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
        Args:
            link_index: index of the link
            pose: target pose of the link in articulation base frame
            initial_qpos: initial qpos to start CLIK
            active_qmask: dof sized integer array, 1 to indicate active joints and 0 for inactive joints, default to all 1s
            max_iterations: number of iterations steps
            dt: iteration step "speed"
            damp: iteration step "damping"
        Returns:
            result: qpos from IK
            success: whether IK is successful
            error: se3 norm error
        """
    def compute_single_link_local_jacobian(self, qpos: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"], link_index: int) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, n]"]:
        """
        Compute the link(body) Jacobian for a single link. It is faster than compute_full_jacobian followed by get_link_jacobian
        """
    def get_link_jacobian(self, link_index: int, local: bool = False) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[6, n]"]:
        """
        Given link index, get the Jacobian. Must be called after compute_full_jacobian.
        Args:
          link_index: index of the link
          local: True for world(spatial) frame; False for link(body) frame
        """
    def get_link_pose(self, link_index: int) -> sapien.pysapien.Pose:
        """
        Given link index, get link pose (in articulation base frame) from forward kinematics. Must be called after compute_forward_kinematics.
        """
    def set_joint_order(self, arg0: list[str]) -> None:
        ...
    def set_link_order(self, arg0: list[str]) -> None:
        ...
