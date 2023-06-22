import numpy as np
import sapien.core as sapien
from sapien.core import renderer as R

from .plugin import Plugin


class TransformWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self.ui_window = None
        self.gizmo = None
        self._gizmo_pose = sapien.Pose()
        self.ghost_objects = []
        self.ik_articulation = None
        self.follow = False
        self.ik_enabled = True
        self.display_ghosts = False

        # self.ui_move_group = None
        self.move_group_joints = []

    def close(self):
        self.reset()

    @property
    def gizmo_matrix(self):
        return self._gizmo_pose.to_transformation_matrix()

    @gizmo_matrix.setter
    def gizmo_matrix(self, v):
        self._gizmo_pose = sapien.Pose(v)
        self.update_ghost_objects()
        self.follow = False

    def notify_selected_entity_change(self):
        # self.refresh_ghost_objects()
        self.clear_ghost_objects()
        if self.viewer.selected_entity is None:
            self._gizmo_pose = sapien.Pose()
            self.follow = False
        else:
            self._gizmo_pose = self.selected_entity.pose
            self.follow = True

            if isinstance(self.selected_entity, sapien.LinkBase):
                if self.ik_articulation == self.selected_entity.get_articulation():
                    return
                self.ik_articulation = self.selected_entity.get_articulation()
                self.pinocchio_model: sapien.PinocchioModel = (
                    self.ik_articulation.create_pinocchio_model()
                )
                self.move_group_joints = [
                    j.name
                    for j in self.ik_articulation.get_joints()
                    if j.get_dof() != 0
                ]
                self.move_group_selection = [True] * len(self.move_group_joints)
                self.ui_move_group.remove_children()
                for i, n in enumerate(self.move_group_joints):
                    self.ui_move_group.append(
                        R.UICheckbox().Label(n).Bind(self.move_group_selection, i)
                    )

    def compute_ik(self):
        if (
            self.selected_entity is not None
            and self.ik_enabled
            and isinstance(self.selected_entity, sapien.LinkBase)
        ):
            link_idx = self.ik_articulation.get_links().index(self.selected_entity)
            mask = np.array(
                [
                    self.move_group_selection[j]
                    for j in range(len(self.move_group_joints))
                ]
            ).astype(int)

            pose = self.ik_articulation.pose.inv() * self._gizmo_pose
            result, success, error = self.pinocchio_model.compute_inverse_kinematics(
                link_idx,
                pose,
                initial_qpos=self.ik_articulation.get_qpos(),
                active_qmask=mask,
                max_iterations=100,
            )
            return result, success, error

    @property
    def scene(self):
        return self.viewer.scene

    @property
    def selected_entity(self):
        return self.viewer.selected_entity

    def clear_ghost_objects(self):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene
        for obj in self.ghost_objects:
            render_scene.remove_node(obj)
        self.ghost_objects = []
        self.viewer.notify_render_update()

    def refresh_ghost_objects(self):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene
        self.clear_ghost_objects()

        if self.selected_entity is None:
            return

        elif isinstance(self.selected_entity, sapien.LinkBase):
            link: sapien.LinkBase = self.selected_entity
            link2world = link.pose
            articulation = link.get_articulation()
            for l in articulation.get_links():
                node = render_scene.add_node()
                for body in l.get_visual_bodies():
                    for obj in body._internal_objects:
                        scale = obj.scale
                        obj2world = sapien.Pose(obj.position, obj.rotation)
                        obj2selected = l.pose.inv() * obj2world
                        new_obj = render_scene.add_object(obj.model, node)
                        new_obj.set_position(obj2selected.p)
                        new_obj.set_rotation(obj2selected.q)
                        new_obj.set_scale(scale)
                        new_obj.transparency = 0.7
                        new_obj.set_segmentation(obj.get_segmentation())
                node.set_position(l.pose.p)
                node.set_rotation(l.pose.q)
                self.ghost_objects.append(node)

        elif isinstance(self.selected_entity, sapien.ActorBase):
            # actors
            actor = self.selected_entity
            actor2world = actor.pose
            node = render_scene.add_node()
            for body in actor.get_visual_bodies():
                for obj in body._internal_objects:
                    scale = obj.scale
                    obj2world = sapien.Pose(obj.position, obj.rotation)
                    obj2selected = actor2world.inv() * obj2world
                    new_obj = render_scene.add_object(obj.model, node)
                    new_obj.set_position(obj2selected.p)
                    new_obj.set_rotation(obj2selected.q)
                    new_obj.set_scale(scale)
                    new_obj.transparency = 0.7
            node.set_position(actor.pose.p)
            node.set_rotation(actor.pose.q)
            self.ghost_objects.append(node)

        self.viewer.notify_render_update()

    def update_ghost_objects(self):
        if self.selected_entity is None:
            return

        if not self.ghost_objects:
            self.refresh_ghost_objects()

        if isinstance(self.selected_entity, sapien.LinkBase):
            if self.ik_enabled:
                # IK
                result, success, error = self.compute_ik()
                self.ik_result = result
                self.ik_success = success
                self.ik_errpr = error
                self.pinocchio_model.compute_forward_kinematics(result)
                for idx, obj in enumerate(self.ghost_objects):
                    pose = (
                        self.selected_entity.get_articulation().pose
                        * self.pinocchio_model.get_link_pose(idx)
                    )
                    obj.set_position(pose.p)
                    obj.set_rotation(pose.q)
            else:
                # no IK
                link: sapien.LinkBase = self.selected_entity
                link2world = link.pose
                articulation = link.get_articulation()
                for l, node in zip(articulation.get_links(), self.ghost_objects):
                    newlink2world = self._gizmo_pose
                    l2world = l.pose
                    l2link = link2world.inv() * l2world
                    newl2world = newlink2world * l2link
                    node.set_position(newl2world.p)
                    node.set_rotation(newl2world.q)
            return

        if isinstance(self.selected_entity, sapien.ActorBase):
            for obj in self.ghost_objects:
                obj.set_position(self._gizmo_pose.p)
                obj.set_rotation(self._gizmo_pose.q)

        self.viewer.notify_render_update()

    def teleport(self, _):
        try:
            if isinstance(self.selected_entity, sapien.LinkBase):
                link: sapien.LinkBase = self.selected_entity
                articulation = link.get_articulation()
                if self.ik_enabled:
                    articulation.set_qpos(self.ik_result)
                else:
                    link2world = link.pose
                    newlink2world = self._gizmo_pose
                    l2world = articulation.pose
                    l2link = link2world.inv() * l2world
                    newl2world = newlink2world * l2link
                    articulation.set_root_pose(newl2world)
            else:
                self.selected_entity.set_pose(self._gizmo_pose)
            self.viewer.notify_render_update()
        except AttributeError:
            pass

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if self.follow and self.selected_entity and self.ghost_objects:
            self._gizmo_pose = self.selected_entity.pose
            self.update_ghost_objects()

        if not self.ui_window:
            self.ui_move_group = R.UISection().Label("Move Group")

            self.gizmo = R.UIGizmo().Bind(self, "gizmo_matrix")
            self.ui_window = (
                R.UIWindow()
                .Label("Transform")
                .Pos(10, 10)
                .Size(400, 400)
                .append(
                    self.gizmo,
                    R.UIConditional()
                    .Bind(lambda: self.selected_entity is not None)
                    .append(
                        R.UIConditional()
                        .Bind(lambda: isinstance(self.selected_entity, sapien.LinkBase))
                        .append(
                            R.UICheckbox().Label("IK").Bind(self, "ik_enabled"),
                            self.ui_move_group,
                        ),
                        R.UIButton().Label("Teleport").Callback(self.teleport),
                    ),
                )
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
