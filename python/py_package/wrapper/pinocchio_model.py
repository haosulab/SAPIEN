import numpy as np
import warnings
from ..pysapien import Pose
from ..pysapien.physx import PhysxArticulation
import platform
from .urdf_exporter import export_kinematic_chain_urdf

try:
    import pinocchio

    class PinocchioModel:
        def __init__(self, urdf_string, gravity):
            self.model: pinocchio.Model = pinocchio.buildModelFromXML(urdf_string)
            self.model.gravity.vector[:] = [*gravity, 0, 0, 0]
            self.data = pinocchio.Data(self.model)

        def set_joint_order(self, names):
            v = np.zeros(self.model.nv, dtype=np.int64)
            count = 0
            for name in names:
                i = self.model.getJointId(name)
                if i >= self.model.njoints:
                    raise ValueError(f"invalid joint name {name}")
                size = self.model.nvs[i]
                qi = self.model.idx_vs[i]
                for s in range(size):
                    v[count] = qi + s
                    count += 1

            self.index_s2p = v

            self.index_p2s = np.zeros_like(self.index_s2p)
            for s, p in enumerate(self.index_s2p):
                self.index_p2s[p] = s

            self.QIDX = np.zeros(
                len(names), dtype=np.int64
            )  # joint position start index
            self.NQ = np.zeros(len(names), dtype=np.int64)  # joint position dim
            self.NV = np.zeros(len(names), dtype=np.int64)  # joint velocity dim
            for N in range(len(names)):
                i = self.model.getJointId(names[N])
                if i >= self.model.njoints:
                    raise ValueError(f"invalid joint name {name}")
                self.NQ[N] = self.model.nqs[i]
                self.NV[N] = self.model.nvs[i]
                self.QIDX[N] = self.model.idx_qs[i]

        def set_link_order(self, names):
            v = []
            for name in names:
                i = self.model.getFrameId(name, pinocchio.BODY)
                if i >= self.model.nframes:
                    raise ValueError(f"invalid joint name {name}")

                v.append(i)

            self.link_id_to_frame_index = np.array(v, dtype=np.int64)

        def q_p2s(self, qint):
            qext = np.zeros(self.model.nv)
            count = 0
            for N in range(len(self.QIDX)):
                start_idx = self.QIDX[N]
                if self.NQ[N] == 1:
                    qext[count] = qint[start_idx]
                elif self.NQ[N] == 2:
                    qext[count] = np.arctan2(qint[start_idx + 1], qint[start_idx])
                elif self.NQ[N] > 2:
                    raise ValueError(
                        f"Unsupported joint in computation. Currently support: fixed, revolute, prismatic"
                    )
                count += self.NV[N]

            assert count == len(self.NV)
            return qext

        def q_s2p(self, qext):
            qint = np.zeros(self.model.nq)
            count = 0
            for N in range(len(self.QIDX)):
                start_idx = self.QIDX[N]
                if self.NQ[N] == 1:
                    qint[start_idx] = qext[count]
                elif self.NQ[N] == 2:
                    qint[start_idx] = np.cos(qext[count])
                    qint[start_idx + 1] = np.sin(qext[count])
                elif self.NQ[N] > 2:
                    raise ValueError(
                        f"Unsupported joint in computation. Currently support: fixed, revolute, prismatic"
                    )
                count += self.NV[N]

            assert count == len(self.NV)
            return qint

        def get_random_qpos(self):
            return self.q_p2s(pinocchio.randomConfiguration(self.model))

        def compute_forward_kinematics(self, qpos):
            """
            Compute and cache forward kinematics. After computation, use get_link_pose to retrieve the computed pose for a specific link.
            """

            pinocchio.forwardKinematics(self.model, self.data, self.q_s2p(qpos))

        def get_link_pose(self, index):
            """
            Given link index, get link pose (in articulation base frame) from forward kinematics. Must be called after compute_forward_kinematics.
            """
            assert 0 <= index < len(self.link_id_to_frame_index)
            frame = int(self.link_id_to_frame_index[index])
            parent_joint = self.model.frames[frame].parent
            link2joint = self.model.frames[frame].placement
            joint2world = self.data.oMi[parent_joint]
            link2world = joint2world * link2joint
            p = link2world.translation
            q = link2world.rotation
            T = np.eye(4)
            T[:3, :3] = q
            T[:3, 3] = p
            return Pose(T)

        def compute_full_jacobian(self, qpos):
            """
            Compute and cache Jacobian for all links
            """
            pinocchio.computeJointJacobians(self.model, self.data, self.q_s2p(qpos))

        def get_link_jacobian(self, index, local=False):
            """
            Given link index, get the Jacobian. Must be called after compute_full_jacobian.

            Args:
              link_index: index of the link
              local: True for world(spatial) frame; False for link(body) frame
            """
            assert 0 <= index < len(self.link_id_to_frame_index)
            frame = int(self.link_id_to_frame_index[index])
            parent_joint = self.model.frames[frame].parent

            link2joint = self.model.frames[frame].placement
            joint2world = self.data.oMi[parent_joint]
            link2world = joint2world * link2joint

            J = pinocchio.getJointJacobian(
                self.model, self.data, parent_joint, pinocchio.ReferenceFrame.WORLD
            )
            if local:
                J = link2world.toActionMatrixInverse() @ J

            return J[:, self.index_s2p]

        def compute_single_link_local_jacobian(self, qpos, index):
            """
            Compute the link(body) Jacobian for a single link. It is faster than compute_full_jacobian followed by get_link_jacobian
            """
            assert 0 <= index < len(self.link_id_to_frame_index)
            frame = int(self.link_id_to_frame_index[index])
            joint = self.model.frames[frame].parent
            link2joint = self.model.frames[frame].placement
            J = pinocchio.computeJointJacobian(
                self.model, self.data, self.q_s2p(qpos), joint
            )
            J = link2joint.toActionMatrixInverse() @ J
            return J[:, self.index_s2p]

        def compute_generalized_mass_matrix(self, qpos):
            return pinocchio.crba(self.model, self.data, self.q_s2p(qpos))[
                self.index_s2p, :
            ][:, self.index_s2p]

        def compute_coriolis_matrix(self, qpos, qvel):
            return pinocchio.computeCoriolisMatrix(
                self.model, self.data, self.q_s2p(qpos), qvel[self.index_p2s]
            )[self.index_s2p, :][:, self.index_s2p]

        def compute_inverse_dynamics(self, qpos, qvel, qacc):
            return pinocchio.rnea(
                self.model,
                self.data,
                self.q_s2p(qpos),
                qvel[self.index_p2s],
                qacc[self.index_p2s],
            )[self.index_s2p]

        def compute_forward_dynamics(self, qpos, qvel, qf):
            return pinocchio.aba(
                self.model,
                self.data,
                self.q_s2p(qpos),
                qvel[self.index_p2s],
                qf[self.index_p2s],
            )[self.index_s2p]

        def compute_inverse_kinematics(
            self,
            link_index,
            pose,
            initial_qpos=None,
            active_qmask=None,
            eps=1e-4,
            max_iterations=1000,
            dt=0.1,
            damp=1e-6,
        ):
            """
            Compute inverse kinematics with CLIK algorithm.
            Details see https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
            Also see bug fix from https://github.com/stack-of-tasks/pinocchio/pull/1963/files
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
            assert 0 <= link_index < len(self.link_id_to_frame_index)
            if initial_qpos is None:
                q = pinocchio.neutral(self.model)
            else:
                q = self.q_s2p(initial_qpos)

            if active_qmask is None:
                mask = np.ones(self.model.nv)
            else:
                mask = np.array(active_qmask)[self.index_p2s]

            mask = np.diag(mask)

            frame = int(self.link_id_to_frame_index[link_index])
            joint = self.model.frames[frame].parent

            T = pose.to_transformation_matrix()
            l2w = pinocchio.SE3()
            l2w.translation[:] = T[:3, 3]
            l2w.rotation[:] = T[:3, :3]

            l2j = self.model.frames[frame].placement
            oMdes = l2w * l2j.inverse()

            best_error = 1e10
            best_q = np.array(q)

            for i in range(max_iterations):
                pinocchio.forwardKinematics(self.model, self.data, q)
                iMd = self.data.oMi[joint].actInv(oMdes)
                err = pinocchio.log6(iMd).vector
                err_norm = np.linalg.norm(err)
                if err_norm < best_error:
                    best_error = err_norm
                    best_q = q

                if err_norm < eps:
                    success = True
                    break

                J = pinocchio.computeJointJacobian(self.model, self.data, q, joint)
                Jlog = pinocchio.Jlog6(iMd.inverse())
                J = -Jlog @ J
                J = J @ mask

                JJt = J @ J.T
                JJt[np.diag_indices_from(JJt)] += damp

                v = -(J.T @ np.linalg.solve(JJt, err))
                q = pinocchio.integrate(self.model, q, v * dt)
            else:
                success = False

            return self.q_p2s(best_q), success, best_error

except ModuleNotFoundError:
    if platform.system() == "Linux":
        from ..pysapien_pinocchio import PinocchioModel
    else:
        warnings.warn(
            "pinnochio package is not installed, robotics functionalities will not be available"
        )
        PinocchioModel = None

except ImportError:
    if platform.system() == "Linux":
        warnings.warn(
            "pinnochio package is broken, fallback to built-in pinocchio. This may be fixed by installing pinocchio via conda instead of pip"
        )
        from ..pysapien_pinocchio import PinocchioModel
    else:
        warnings.warn(
            "pinnochio package is broken, robotics functionalities will not be available"
        )
        PinocchioModel = None


def _create_pinocchio_model(
    articulation: PhysxArticulation, gravity=[0, 0, -9.81]
) -> PinocchioModel:
    xml = export_kinematic_chain_urdf(articulation, force_fix_root=True)
    model = PinocchioModel(xml, gravity)
    model.set_joint_order(
        [f"joint_{j.child_link.index}" for j in articulation.active_joints]
    )
    model.set_link_order([f"link_{l.index}" for l in articulation.links])
    return model


PhysxArticulation.create_pinocchio_model = _create_pinocchio_model
