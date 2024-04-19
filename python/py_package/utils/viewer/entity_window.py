from .plugin import Plugin, copy_to_clipboard
from sapien import internal_renderer as R
import sapien
import numpy as np


class EntityWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self.ui_window = None

    def close(self):
        self.reset()

    @property
    def selected_entity(self):
        return self.viewer.selected_entity

    def set_actor_pose(self, pose):
        try:
            self.selected_entity.pose = pose
        except AttributeError:
            pass

    def enable_collision_visual(self, entity=None):
        if entity is None:
            entity = self.selected_entity
        if entity is None:
            return

        for c in entity.components:
            if isinstance(c, sapien.render.RenderBodyComponent):
                if c.name == "Collision":
                    return

                c.disable()

        new_visual = sapien.render.RenderBodyComponent()
        new_visual.disable_render_id()  # avoid it interfere with visual id counting

        red_mat = sapien.render.RenderMaterial(base_color=[1, 0, 0, 1])
        green_mat = sapien.render.RenderMaterial(base_color=[0, 1, 0, 1])
        blue_mat = sapien.render.RenderMaterial(base_color=[0, 0, 1, 1])
        new_visual.name = "Collision"

        for c in entity.components:
            if isinstance(c, sapien.physx.PhysxRigidBaseComponent):
                for s in c.collision_shapes:
                    if isinstance(s, sapien.physx.PhysxCollisionShapeSphere):
                        vs = sapien.render.RenderShapeSphere(s.radius, blue_mat)

                    elif isinstance(s, sapien.physx.PhysxCollisionShapeBox):
                        vs = sapien.render.RenderShapeBox(s.half_size, blue_mat)

                    elif isinstance(s, sapien.physx.PhysxCollisionShapeCapsule):
                        vs = sapien.render.RenderShapeCapsule(
                            s.radius, s.half_length, blue_mat
                        )

                    elif isinstance(s, sapien.physx.PhysxCollisionShapeConvexMesh):
                        vs = sapien.render.RenderShapeTriangleMesh(
                            s.vertices,
                            s.triangles,
                            np.zeros((0, 3)),
                            np.zeros((0, 2)),
                            green_mat,
                        )
                        vs.scale = s.scale

                    elif isinstance(s, sapien.physx.PhysxCollisionShapeTriangleMesh):
                        vs = sapien.render.RenderShapeTriangleMesh(
                            s.vertices,
                            s.triangles,
                            np.zeros((0, 3)),
                            np.zeros((0, 2)),
                            red_mat,
                        )
                        vs.scale = s.scale

                    elif isinstance(s, sapien.physx.PhysxCollisionShapePlane):
                        vs = sapien.render.RenderShapePlane([1, 1e4, 1e4], blue_mat)

                    elif isinstance(s, sapien.physx.PhysxCollisionShapeCylinder):
                        vs = sapien.render.RenderShapeCylinder(
                            s.radius, s.half_length, green_mat
                        )

                    else:
                        raise Exception(
                            "invalid collision shape, this code should be unreachable."
                        )

                    vs.local_pose = s.local_pose

                    new_visual.attach(vs)

        entity.add_component(new_visual)
        new_visual.set_property("shadeFlat", 1)
        self.viewer.notify_render_update()

    def disable_collision_visual(self, entity=None):
        if entity is None:
            entity = self.selected_entity
        if entity is None:
            return

        for c in entity.components:
            if isinstance(c, sapien.render.RenderBodyComponent):
                if c.name == "Collision":
                    entity.remove_component(c)
                    continue

                c.enable()

        self.viewer.notify_render_update()

    def create_collision_shapes(self, entity):
        render_scene: R.Scene = self.viewer.system._internal_scene
        # TODO

    def copy_pose(self, _):
        copy_to_clipboard(str(self.selected_entity.pose))

    def disconnect(self, c):
        self.viewer.select_entity(None)
        c.set_parent(None)

    def build(self):
        if self.viewer.render_scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.ui_window = R.UIWindow().Pos(10, 10).Size(400, 400).Label("Entity")
        else:
            self.ui_window.remove_children()

        if self.selected_entity is None:
            self.ui_window.append(R.UIDisplayText().Text("No actor/entity selected."))
            return

        p = self.selected_entity.pose.p
        q = self.selected_entity.pose.q
        self.ui_window.append(
            R.UIDisplayText().Text("Name: {}".format(self.selected_entity.name)),
            R.UIDisplayText().Text("Id: {}".format(self.selected_entity.per_scene_id)),
            R.UIInputFloat3()
            .Label("pose.p")
            .Id("xyz")
            .Value(p)
            .Callback(lambda v: self.set_actor_pose(sapien.Pose(v.value, q))),
            R.UIInputFloat4().Value(q).ReadOnly(True).Label("pose.q"),
            R.UIButton().Label("Copy Pose").Callback(self.copy_pose),
        )

        for cid, c in enumerate(self.selected_entity.components):
            name = f"({c.name})" if c.name else ""
            if isinstance(c, sapien.render.RenderBodyComponent):
                section = R.UISection().Label(f"Render Body {name}##{cid}")
                self.ui_window.append(section)
                for idx, s in enumerate(c.render_shapes):
                    if s.__class__.__name__ == "RenderShapePlane":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Plane")
                            .Id("visual{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Name: {s.name}"),
                                R.UIDisplayText().Text(f"Id: {s.per_scene_id}"),
                                R.UIInputFloat3()
                                .Label("Scale")
                                .ReadOnly(True)
                                .Value(s.scale),
                            )
                        )
                    if s.__class__.__name__ == "RenderShapeBox":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Box")
                            .Id("visual{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Name: {s.name}"),
                                R.UIDisplayText().Text(f"Id: {s.per_scene_id}"),
                                R.UIInputFloat3()
                                .Label("Half Size")
                                .ReadOnly(True)
                                .Value(s.half_size),
                            )
                        )
                    if s.__class__.__name__ == "RenderShapeSphere":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Sphere")
                            .Id("visual{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Name: {s.name}"),
                                R.UIDisplayText().Text(f"Id: {s.per_scene_id}"),
                                R.UIInputFloat()
                                .Label("Radius")
                                .ReadOnly(True)
                                .Value(s.radius),
                            )
                        )
                    if s.__class__.__name__ == "RenderShapeCapsule":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Capsule")
                            .Id("visual{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Name: {s.name}"),
                                R.UIDisplayText().Text(f"Id: {s.per_scene_id}"),
                                R.UIInputFloat()
                                .Label("Radius")
                                .ReadOnly(True)
                                .Value(s.radius),
                                R.UIInputFloat()
                                .Label("Half Length")
                                .ReadOnly(True)
                                .Value(s.half_length),
                            )
                        )
                    if s.__class__.__name__ == "RenderShapeCylinder":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Cylinder")
                            .Id("visual{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Name: {s.name}"),
                                R.UIDisplayText().Text(f"Id: {s.per_scene_id}"),
                                R.UIInputFloat()
                                .Label("Radius")
                                .ReadOnly(True)
                                .Value(s.radius),
                                R.UIInputFloat()
                                .Label("Half Length")
                                .ReadOnly(True)
                                .Value(s.half_length),
                            )
                        )
                    if s.__class__.__name__ == "RenderShapeTriangleMesh":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Triangle Mesh")
                            .Id("visual{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Name: {s.name}"),
                                R.UIDisplayText().Text(f"Id: {s.per_scene_id}"),
                            )
                            # .append(
                            #     R.UIInputFloat()
                            #     .Label("filename")
                            #     .ReadOnly(True)
                            #     .Value(s.filename),
                            #     R.UIInputFloat()
                            #     .Label("Scale")
                            #     .ReadOnly(True)
                            #     .Value(s.scale),
                            # )
                        )

                    section.append(shape_info)

            elif isinstance(c, sapien.physx.PhysxRigidBaseComponent):
                if isinstance(c, sapien.physx.PhysxRigidStaticComponent):
                    section = R.UISection().Label("Static Body")
                    self.ui_window.append(section)

                if isinstance(c, sapien.physx.PhysxRigidDynamicComponent) or isinstance(
                    c, sapien.physx.PhysxArticulationLinkComponent
                ):
                    section = R.UISection().Label(
                        "Dynamic Body"
                        if isinstance(c, sapien.physx.PhysxRigidDynamicComponent)
                        else "Articulation Link"
                    )
                    self.ui_window.append(section)
                    section.append(
                        R.UIInputFloat().Label("Mass").ReadOnly(True).Value(c.mass),
                    )
                    section.append(
                        R.UIInputFloat3()
                        .Label("Inertia")
                        .ReadOnly(True)
                        .Value(c.inertia),
                    )
                    section.append(
                        R.UICheckbox()
                        .Label("Auto Compute Mass")
                        .Checked(c.auto_compute_mass),
                    )

                if (
                    isinstance(c, sapien.physx.PhysxArticulationLinkComponent)
                    and c.parent
                ):
                    section.append(
                        R.UISameLine().append(
                            R.UIButton()
                            .Label("Select Parent")
                            .Callback(
                                (
                                    lambda c: lambda _: self.viewer.select_entity(
                                        c.parent.entity if c.parent else None
                                    )
                                )(c)
                            ),
                            R.UIButton()
                            .Label("Disconnect")
                            .Callback((lambda c: lambda _: self.disconnect(c))(c)),
                        )
                    )

                for idx, s in enumerate(c.collision_shapes):
                    if s.__class__.__name__ == "PhysxCollisionShapePlane":
                        shape_info = (
                            R.UITreeNode().Label("Plane").Id("collision{}".format(idx))
                        )
                    if s.__class__.__name__ == "PhysxCollisionShapeBox":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Box")
                            .Id("collision{}".format(idx))
                            .append(
                                R.UIInputFloat3()
                                .Label("Half Size")
                                .ReadOnly(True)
                                .Value(s.half_size),
                                R.UIDisplayText().Text(f"Density: {s.density}"),
                            )
                        )
                    if s.__class__.__name__ == "PhysxCollisionShapeCapsule":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Capsule")
                            .Id("collision{}".format(idx))
                            .append(
                                R.UIInputFloat()
                                .Label("Radius")
                                .ReadOnly(True)
                                .Value(s.radius),
                                R.UIInputFloat()
                                .Label("Half Length")
                                .ReadOnly(True)
                                .Value(s.half_length),
                                R.UIDisplayText().Text(f"Density: {s.density}"),
                            )
                        )
                    if s.__class__.__name__ == "PhysxCollisionShapeCylinder":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Cylinder")
                            .Id("collision{}".format(idx))
                            .append(
                                R.UIInputFloat()
                                .Label("Radius")
                                .ReadOnly(True)
                                .Value(s.radius),
                                R.UIInputFloat()
                                .Label("Half Length")
                                .ReadOnly(True)
                                .Value(s.half_length),
                                R.UIDisplayText().Text(f"Density: {s.density}"),
                            )
                        )
                    if s.__class__.__name__ == "PhysxCollisionShapeSphere":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Sphere")
                            .Id("collision{}".format(idx))
                            .append(
                                R.UIInputFloat()
                                .Label("Radius")
                                .ReadOnly(True)
                                .Value(s.radius),
                                R.UIDisplayText().Text(f"Density: {s.density}"),
                            )
                        )
                    if s.__class__.__name__ == "PhysxCollisionShapeConvexMesh":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Convex Mesh")
                            .Id("collision{}".format(idx))
                            .append(
                                R.UIDisplayText().Text(f"Density: {s.density}"),
                            )
                        )
                    if s.__class__.__name__ == "PhysxCollisionShapeTriangleMesh":
                        shape_info = (
                            R.UITreeNode()
                            .Label("Triangle Mesh")
                            .Id("collision{}".format(idx))
                        )

                    s: sapien.physx.PhysxCollisionShape
                    c0, c1, c2, c3 = s.collision_groups
                    mat = s.physical_material

                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Contact offset: {:.3g}".format(s.contact_offset)
                        ),
                        R.UIDisplayText().Text(
                            "Rest offset: {:.3g}".format(s.rest_offset)
                        ),
                        R.UIDisplayText().Text(
                            "Patch radius: {:.3g}".format(s.patch_radius)
                        ),
                        R.UIDisplayText().Text(
                            "Min path radius: {:.3g}".format(s.min_patch_radius)
                        ),
                        # R.UICheckbox().Label("Is trigger").Checked(s.is_trigger),
                        R.UIDisplayText().Text(
                            "Static friction: {:.3g}".format(mat.get_static_friction())
                        ),
                        R.UIDisplayText().Text(
                            "Dynamic friction: {:.3g}".format(
                                mat.get_dynamic_friction()
                            )
                        ),
                        R.UIDisplayText().Text(
                            "Restitution: {:.3g}".format(mat.get_restitution())
                        ),
                        R.UIDisplayText().Text("Collision groups:"),
                        R.UIDisplayText().Text("  0x{:08x}  0x{:08x}".format(c0, c1)),
                        R.UIDisplayText().Text("  0x{:08x}  0x{:08x}".format(c2, c3)),
                    )

                    section.append(shape_info)

                # only supported in single scene mode
                if len(self.viewer.scenes) == 1:
                    section.append(
                        R.UISameLine().append(
                            R.UIButton()
                            .Label("Show")
                            .Callback(lambda _: self.enable_collision_visual()),
                            R.UIButton()
                            .Label("Hide")
                            .Callback(lambda _: self.disable_collision_visual()),
                            R.UIDisplayText().Text("Collision"),
                        )
                    )

            elif isinstance(c, sapien.physx.PhysxDriveComponent):
                c: sapien.physx.PhysxDriveComponent
                section = R.UISection().Label("6D Drive")
                self.ui_window.append(section)
                section.append(
                    R.UIButton()
                    .Label("Select Parent")
                    .Callback(
                        (
                            lambda c: lambda _: self.viewer.select_entity(
                                c.parent.entity if c.parent else None
                            )
                        )(c)
                    ),
                )
            else:
                section = R.UISection().Label(c.__class__.__name__)
                self.ui_window.append(section)
                section.append(
                    R.UIDisplayText().Text("UI for this component is not implemneted")
                )

        #     for body_idx, body in enumerate(bodies):
        #         body_info = (
        #             R.UITreeNode().Label(body.type).Id("visual{}".format(body_idx))
        #         )
        #         body_section.append(body_info)

        #         if body.type == "sphere":
        #             body_info.append(
        #                 R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius))
        #             )
        #         elif body.type == "capsule":
        #             body_info.append(
        #                 R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius)),
        #                 R.UIDisplayText().Text(
        #                     "Half length: {:.3g}".format(body.half_length)
        #                 ),
        #             )
        #         elif body.type == "box":
        #             x, y, z = body.half_size
        #             body_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Half size: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )
        #         elif body.type == "mesh":
        #             x, y, z = body.scale
        #             body_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )
        #         body_info.append(
        #             R.UIDisplayText().Text("Visual id: {}".format(body.visual_id))
        #         )
        #         shapes = body.get_render_shapes()
        #         for shape_idx, shape in enumerate(shapes):
        #             mat = shape.material
        #             dtex = (
        #                 (mat.diffuse_texture_filename or "(has texture)")
        #                 if mat.diffuse_texture
        #                 else "(no texture)"
        #             )
        #             rtex = (
        #                 (mat.roughness_texture_filename or "(has texture)")
        #                 if mat.roughness_texture
        #                 else "(no texture)"
        #             )
        #             mtex = (
        #                 (mat.metallic_texture_filename or "(has texture)")
        #                 if mat.metallic_texture
        #                 else "(no texture)"
        #             )
        #             ntex = (
        #                 (mat.normal_texture_filename or "(has texture)")
        #                 if mat.normal_texture
        #                 else "(no texture)"
        #             )
        #             etex = (
        #                 (mat.emission_texture_filename or "(has texture)")
        #                 if mat.emission_texture
        #                 else "(no texture)"
        #             )

        # if isinstance(self.selected_entity, sapien.ActorBase):
        #     actor = self.selected_entity
        #     actor: sapien.ActorBase
        #     p = actor.pose.p
        #     q = actor.pose.q
        #     self.ui_window.append(
        #         R.UIDisplayText().Text("Name: {}".format(actor.name)),
        #         R.UIDisplayText().Text("Class: {}".format(actor.classname)),
        #         R.UIDisplayText().Text("Id: {}".format(actor.id)),
        #         R.UIDisplayText().Text("Position XYZ"),
        #         R.UIInputFloat3()
        #         .Id("xyz")
        #         .Value(p)
        #         .Callback(lambda v: self.set_actor_pose(sapien.Pose(v.value, q))),
        #         R.UIDisplayText().Text("Rotation WXYZ"),
        #         R.UIInputFloat4().Value(q).ReadOnly(True),
        #         R.UIButton().Label("Copy Pose").Callback(self.copy_pose),
        #         R.UICheckbox().Label("Show Collision").Bind(self, "show_collision"),
        #     )

        #     if actor.classname in ["Actor", "Link"]:
        #         self.ui_window.append(
        #             R.UIDisplayText().Text("Mass"),
        #             R.UIInputFloat().Value(actor.mass).ReadOnly(True),
        #         )

        #     # collision shapes
        #     shape_section = R.UISection().Label("Collision Shapes")
        #     self.ui_window.append(shape_section)
        #     shapes = actor.get_collision_shapes()
        #     for shape_idx, shape in enumerate(shapes):
        #         c0, c1, c2, c3 = shape.get_collision_groups()
        #         shape_pose = shape.get_local_pose()
        #         mat = shape.get_physical_material()

        #         shape_info = (
        #             R.UITreeNode()
        #             .Label(shape.type)
        #             .Id("shape{}".format(shape_idx))
        #             .append(
        #                 R.UIDisplayText().Text(
        #                     "Contact offset: {:.3g}".format(shape.contact_offset)
        #                 ),
        #                 R.UIDisplayText().Text(
        #                     "Rest offset: {:.3g}".format(shape.rest_offset)
        #                 ),
        #                 R.UIDisplayText().Text(
        #                     "Patch radius: {:.3g}".format(shape.patch_radius)
        #                 ),
        #                 R.UIDisplayText().Text(
        #                     "Min path radius: {:.3g}".format(shape.min_patch_radius)
        #                 ),
        #                 R.UICheckbox().Label("Is trigger").Checked(shape.is_trigger),
        #                 R.UIDisplayText().Text(
        #                     "Static friction: {:.3g}".format(mat.get_static_friction())
        #                 ),
        #                 R.UIDisplayText().Text(
        #                     "Dynamic friction: {:.3g}".format(
        #                         mat.get_dynamic_friction()
        #                     )
        #                 ),
        #                 R.UIDisplayText().Text(
        #                     "Restitution: {:.3g}".format(mat.get_restitution())
        #                 ),
        #                 R.UIDisplayText().Text("Collision groups:"),
        #                 R.UIDisplayText().Text("  0x{:08x}  0x{:08x}".format(c0, c1)),
        #                 R.UIDisplayText().Text("  0x{:08x}  0x{:08x}".format(c2, c3)),
        #                 R.UIDisplayText().Text("Local position XYZ"),
        #                 R.UIInputFloat3()
        #                 .Id("shape_pos{}".format(shape_idx))
        #                 .Value(shape_pose.p)
        #                 .ReadOnly(True),
        #                 R.UIDisplayText().Text("Local rotation WXYZ"),
        #                 R.UIInputFloat4()
        #                 .Id("shape_rot{}".format(shape_idx))
        #                 .Value(shape_pose.q)
        #                 .ReadOnly(True),
        #             )
        #         )

        #         shape_section.append(shape_info)

        #         if shape.type == "sphere":
        #             shape_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Sphere radius: {:.3g}".format(shape.geometry.radius)
        #                 )
        #             )
        #         elif shape.type == "capsule":
        #             shape_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Capsule radius: {:.3g}".format(shape.geometry.radius)
        #                 ),
        #                 R.UIDisplayText().Text(
        #                     "Capsule half length: {:.3g}".format(
        #                         shape.geometry.half_length
        #                     )
        #                 ),
        #             )
        #         elif shape.type == "box":
        #             x, y, z = shape.geometry.half_size
        #             shape_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Box half size: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )
        #         elif shape.type == "convex_mesh":
        #             x, y, z = shape.geometry.scale
        #             shape_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Mesh scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )
        #         elif shape.type == "nonconvex_mesh":
        #             x, y, z = shape.geometry.scale
        #             shape_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Mesh scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )

        #     # render shapes
        #     body_section = R.UISection().Label("Visual Bodies")
        #     self.ui_window.append(body_section)
        #     bodies = actor.get_visual_bodies()
        #     for body_idx, body in enumerate(bodies):
        #         body_info = (
        #             R.UITreeNode().Label(body.type).Id("visual{}".format(body_idx))
        #         )
        #         body_section.append(body_info)

        #         if body.type == "sphere":
        #             body_info.append(
        #                 R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius))
        #             )
        #         elif body.type == "capsule":
        #             body_info.append(
        #                 R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius)),
        #                 R.UIDisplayText().Text(
        #                     "Half length: {:.3g}".format(body.half_length)
        #                 ),
        #             )
        #         elif body.type == "box":
        #             x, y, z = body.half_size
        #             body_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Half size: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )
        #         elif body.type == "mesh":
        #             x, y, z = body.scale
        #             body_info.append(
        #                 R.UIDisplayText().Text(
        #                     "Scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
        #                 )
        #             )
        #         body_info.append(
        #             R.UIDisplayText().Text("Visual id: {}".format(body.visual_id))
        #         )
        #         shapes = body.get_render_shapes()
        #         for shape_idx, shape in enumerate(shapes):
        #             mat = shape.material
        #             dtex = (
        #                 (mat.diffuse_texture_filename or "(has texture)")
        #                 if mat.diffuse_texture
        #                 else "(no texture)"
        #             )
        #             rtex = (
        #                 (mat.roughness_texture_filename or "(has texture)")
        #                 if mat.roughness_texture
        #                 else "(no texture)"
        #             )
        #             mtex = (
        #                 (mat.metallic_texture_filename or "(has texture)")
        #                 if mat.metallic_texture
        #                 else "(no texture)"
        #             )
        #             ntex = (
        #                 (mat.normal_texture_filename or "(has texture)")
        #                 if mat.normal_texture
        #                 else "(no texture)"
        #             )
        #             etex = (
        #                 (mat.emission_texture_filename or "(has texture)")
        #                 if mat.emission_texture
        #                 else "(no texture)"
        #             )

        #             body_info.append(
        #                 R.UITreeNode()
        #                 .Label("material {}##{}".format(shape_idx, body_idx))
        #                 .append(
        #                     R.UIInputFloat4()
        #                     .Label("Diffuse")
        #                     .ReadOnly(True)
        #                     .Value(mat.base_color),
        #                     R.UIInputText()
        #                     .ReadOnly(True)
        #                     .Value(dtex)
        #                     .Label("##dtex{}_{}".format(shape_idx, body_idx)),
        #                     R.UIInputFloat4()
        #                     .Label("Emission")
        #                     .ReadOnly(True)
        #                     .Value(mat.emission),
        #                     R.UIInputText()
        #                     .ReadOnly(True)
        #                     .Value(etex)
        #                     .Label("##etex{}_{}".format(shape_idx, body_idx)),
        #                     R.UIInputFloat()
        #                     .Label("Roughness")
        #                     .ReadOnly(True)
        #                     .Value(mat.roughness),
        #                     R.UIInputText()
        #                     .ReadOnly(True)
        #                     .Value(rtex)
        #                     .Label("##rtex{}_{}".format(shape_idx, body_idx)),
        #                     R.UIInputFloat()
        #                     .Label("Metallic")
        #                     .ReadOnly(True)
        #                     .Value(mat.metallic),
        #                     R.UIInputText()
        #                     .ReadOnly(True)
        #                     .Value(mtex)
        #                     .Label("##mtex{}_{}".format(shape_idx, body_idx)),
        #                     R.UIInputFloat()
        #                     .Label("Specular")
        #                     .ReadOnly(True)
        #                     .Value(mat.specular),
        #                     R.UIDisplayText().Text("Normal map"),
        #                     R.UIInputText()
        #                     .ReadOnly(True)
        #                     .Value(ntex)
        #                     .Label("##ntex{}_{}".format(shape_idx, body_idx)),
        #                 )
        #             )

        # if isinstance(self.selected_entity, sapien.LightEntity):
        #     light = self.selected_entity
        #     p = light.pose.p
        #     q = light.pose.q
        #     self.ui_window.append(
        #         R.UIDisplayText().Text("Name: {}".format(light.name)),
        #         R.UIDisplayText().Text("Class: {}".format(light.classname)),
        #     )

        #     def set_shadow(light, enable):
        #         light.shadow = enable

        #     self.ui_window.append(
        #         R.UICheckbox()
        #         .Label("Shadow")
        #         .Checked(light.shadow)
        #         .Callback((lambda light: lambda p: set_shadow(light, p.checked))(light))
        #     )

        #     self.ui_window.append(
        #         R.UIDisplayText().Text("Position XYZ"),
        #         R.UIInputFloat3()
        #         .Id("xyz")
        #         .Value(p)
        #         .Callback(lambda v: self.set_actor_pose(sapien.Pose(v.value, q))),
        #         R.UIDisplayText().Text("Rotation WXYZ"),
        #         R.UIInputFloat4().Value(q).ReadOnly(True),
        #         R.UIButton().Label("Copy Pose").Callback(self.copy_pose),
        #     )
        #     if hasattr(light, "shadow_near"):
        #         self.ui_window.append(
        #             R.UIInputFloat()
        #             .Label("Near")
        #             .Value(light.shadow_near)
        #             .ReadOnly(True),
        #             R.UIInputFloat()
        #             .Label("Far")
        #             .Value(light.shadow_far)
        #             .ReadOnly(True),
        #         )

        #     if light.classname == "PointLightEntity":
        #         light: PointLightEntity
        #         pass
        #     elif light.classname == "DirectionalLightEntity":
        #         self.ui_window.append(
        #             R.UIInputFloat()
        #             .Label("Half Size")
        #             .Value(light.shadow_half_size)
        #             .ReadOnly(True),
        #         )
        #         pass
        #     elif light.classname == "SpotLightEntity":
        #         pass

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []

    def notify_scene_change(self):
        self.reset()
