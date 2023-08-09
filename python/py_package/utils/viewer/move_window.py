import numpy as np
import sapien
from sapien import internal_renderer as R

from .plugin import Plugin


class MoveWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self.ui_window = None
        self.gizmo = None
        self.move_group = None

    def refresh_ik(self):
        if (
            self.selected_entity
            and isinstance(self.selected_entity, LinkBase)
            and self.ik_enabled
        ):
            self.pinocchio_model = (
                self.selected_entity.get_articulation().create_pinocchio_model()
            )
            self.gizmo.Matrix(self.selected_entity.pose.to_transformation_matrix())
            self.move_group_joints = [
                j.name
                for j in self.selected_entity.get_articulation().get_joints()
                if j.get_dof() != 0
            ]
            for n in self.move_group_joints:
                if n not in self.move_group_selection:
                    self.move_group_selection[n] = True

            self.move_group.remove_children()

            def select_move_group(name, select):
                self.move_group_selection[name] = select

            for n in self.move_group_joints:
                self.move_group.append(
                    R.UICheckbox()
                    .Label(n)
                    .Checked(self.move_group_selection[n])
                    .Callback((lambda n: lambda c: select_move_group(n, c.checked))(n))
                )

    @property
    def scene(self):
        return self.viewer.scene

    @property
    def selected_entity(self):
        return self.viewer.active_entity

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if not self.ui_window:
            self.gizmo = R.UIGizmo().Matrix(np.eye(4))
            self.ui_window = (
                R.UIWindow()
                .Label("Move")
                .Pos(10, 10)
                .Size(400, 400)
                # .append(
                #     R.UISameLine().append(
                #         R.UICheckbox()
                #         .Label("Enable IK")
                #         .Callback(lambda c: self.enable_ik(c.checked)),
                #         R.UIButton().Label("Go!").Callback(self.execute_ik),
                #     ),
                #     self.move_group,
                # )
                .append(self.gizmo)
            )

        proj = self.viewer.window.get_camera_projection_matrix()
        view = (
            (
                self.viewer.window.get_camera_pose()
                * sapien.Pose([0, 0, 0], [-0.5, -0.5, 0.5, 0.5])
            )
            .inv()
            .to_transformation_matrix()
        )
        self.gizmo.CameraMatrices(view, proj)

        if self.selected_entity is not None:
            self.gizmo.Matrix(self.selected_entity.pose.to_transformation_matrix())
        else:
            self.gizmo.Matrix(np.eye(4))

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []
