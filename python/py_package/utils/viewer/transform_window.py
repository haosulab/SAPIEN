import numpy as np
import sapien
from sapien import internal_renderer as R

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
        self.ik_enabled = True

        self.ik_errpr = None
        self.ik_result = None
        self.ik_success = False

        self.follow = False
        self.display_ghosts = False
        self.enabled = False
        self.ui_move_group = None
        self.move_group_joints = []
        self.move_group_selection = []
        self.pinocchio_model = None

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

    def get_articulation(self, entity):
        if not entity:
            return None

        for c in entity.components:
            if isinstance(c, sapien.physx.PhysxArticulationLinkComponent):
                return c.articulation
        return None

    def notify_selected_entity_change(self):
        if not self.scene:
            return

        self.clear_ghost_objects()
        if self.viewer.selected_entity is None:
            self._gizmo_pose = sapien.Pose()
            self.follow = False
        else:
            self._gizmo_pose = self.selected_entity.pose
            self.follow = True

            art = self.get_articulation(self.selected_entity)
            if art is not None:
                if self.ik_articulation == art:
                    return
                self.ik_articulation = art
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
            and self.get_articulation(self.selected_entity)
        ):
            link_idx = self.ik_articulation.get_links().index(
                next(
                    c
                    for c in self.selected_entity.components
                    if isinstance(c, sapien.physx.PhysxArticulationLinkComponent)
                )
            )
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
        render_scene: R.Scene = self.viewer.render_scene
        for obj in self.ghost_objects:
            render_scene.remove_node(obj)
        self.ghost_objects = []
        self.viewer.notify_render_update()

    def refresh_ghost_objects(self):
        render_scene: R.Scene = self.viewer.render_scene
        self.clear_ghost_objects()

        if self.selected_entity is None:
            return

        render_body = None
        articulation_link = None
        for c in self.selected_entity.components:
            if isinstance(c, sapien.render.RenderBodyComponent):
                render_body = c
            if isinstance(c, sapien.physx.PhysxArticulationLinkComponent):
                articulation_link = c

        if render_body is None:
            return

        elif articulation_link is not None:
            link: sapien.LinkBase = self.selected_entity
            link2world = link.pose
            articulation = articulation_link.articulation
            for l in articulation.get_links():
                new_node = render_scene.add_node()
                for body in [
                    c
                    for c in l.entity.components
                    if isinstance(c, sapien.render.RenderBodyComponent)
                ]:
                    render_node = body._internal_node
                    for obj in render_node.children:
                        new_obj = render_scene.add_object(obj.model, new_node)
                        new_obj.set_position(obj.position)
                        new_obj.set_rotation(obj.rotation)
                        new_obj.set_scale(obj.scale)
                        new_obj.transparency = 0.7
                        new_obj.set_segmentation(obj.get_segmentation())

                new_node.set_position(l.pose.p)
                new_node.set_rotation(l.pose.q)
                self.ghost_objects.append(new_node)

        else:
            entity2world = self.selected_entity.pose
            render_node = render_body._internal_node
            new_node = render_scene.add_node()

            for obj in render_node.children:
                new_obj = render_scene.add_object(obj.model, new_node)
                new_obj.set_position(obj.position)
                new_obj.set_rotation(obj.rotation)
                new_obj.set_scale(obj.scale)
                new_obj.transparency = 0.7

            new_node.set_position(self.selected_entity.pose.p)
            new_node.set_rotation(self.selected_entity.pose.q)
            self.ghost_objects.append(new_node)

        self.viewer.notify_render_update()

    def update_ghost_objects(self):
        if self.selected_entity is None:
            return

        if not self.ghost_objects:
            self.refresh_ghost_objects()

        art = self.get_articulation(self.selected_entity)
        if art is not None:
            if self.ik_enabled:
                # IK
                result, success, error = self.compute_ik()
                self.ik_result = result
                self.ik_success = success
                self.ik_errpr = error
                self.pinocchio_model.compute_forward_kinematics(result)
                for idx, obj in enumerate(self.ghost_objects):
                    pose = art.pose * self.pinocchio_model.get_link_pose(idx)
                    obj.set_position(pose.p)
                    obj.set_rotation(pose.q)
            else:
                link = self.selected_entity
                link2world = link.pose
                for l, node in zip(art.get_links(), self.ghost_objects):
                    newlink2world = self._gizmo_pose
                    l2world = l.pose
                    l2link = link2world.inv() * l2world
                    newl2world = newlink2world * l2link
                    node.set_position(newl2world.p)
                    node.set_rotation(newl2world.q)
            return

        else:
            for node in self.ghost_objects:
                node.set_position(self._gizmo_pose.p)
                node.set_rotation(self._gizmo_pose.q)

        self.viewer.notify_render_update()

    def teleport(self, _):
        try:
            art = self.get_articulation(self.selected_entity)
            if art:
                link: sapien.LinkBase = self.selected_entity
                if self.ik_enabled:
                    art.set_qpos(self.ik_result)
                else:
                    link2world = link.pose
                    newlink2world = self._gizmo_pose
                    l2world = art.pose
                    l2link = link2world.inv() * l2world
                    newl2world = newlink2world * l2link
                    art.set_root_pose(newl2world)
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
                    R.UICheckbox().Label("Enabled").Bind(self, "enabled"),
                    R.UIConditional()
                    .Bind(self, "enabled")
                    .append(
                        self.gizmo,
                        R.UIConditional()
                        .Bind(lambda: self.selected_entity is not None)
                        .append(
                            R.UIConditional()
                            .Bind(
                                lambda: self.get_articulation(self.selected_entity)
                                is not None
                            )
                            .append(
                                R.UICheckbox().Label("IK").Bind(self, "ik_enabled"),
                                self.ui_move_group,
                            ),
                            R.UIButton().Label("Teleport").Callback(self.teleport),
                        ),
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
