from .plugin import Plugin
from sapien.core import renderer as R
import sapien.core as sapien


class ActorWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self.ui_window = None

    def close(self):
        self.reset()

    @property
    def scene(self):
        return self.viewer.scene

    @property
    def selected_entity(self):
        return self.viewer.selected_entity

    def set_actor_pose(self, pose):
        # TODO: handle articulation
        try:
            self.selected_entity.set_pose(pose)
        except AttributeError:
            pass

    @property
    def show_collision(self):
        if self.selected_entity is None:
            return False
        if isinstance(self.selected_entity, sapien.ActorBase):
            return self.selected_entity.is_rendering_collision()

    @show_collision.setter
    def show_collision(self, v):
        if self.selected_entity is None:
            return
        if isinstance(self.selected_entity, sapien.ActorBase):
            return self.selected_entity.render_collision(v)

    def copy_pose(self, _):
        import pyperclip

        if self.selected_entity is not None:
            pyperclip.copy(str(self.selected_entity.pose))

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.ui_window = (
                R.UIWindow().Pos(10, 10).Size(400, 400).Label("Actor/Entity")
            )
        else:
            self.ui_window.remove_children()

        if self.selected_entity is None:
            self.ui_window.append(R.UIDisplayText().Text("No actor/entity selected."))
            return

        if isinstance(self.selected_entity, sapien.ActorBase):
            actor = self.selected_entity
            actor: sapien.ActorBase
            p = actor.pose.p
            q = actor.pose.q
            self.ui_window.append(
                R.UIDisplayText().Text("Name: {}".format(actor.name)),
                R.UIDisplayText().Text("Class: {}".format(actor.classname)),
                R.UIDisplayText().Text("Id: {}".format(actor.id)),
                R.UIDisplayText().Text("Position XYZ"),
                R.UIInputFloat3()
                .Id("xyz")
                .Value(p)
                .Callback(lambda v: self.set_actor_pose(sapien.Pose(v.value, q))),
                R.UIDisplayText().Text("Rotation WXYZ"),
                R.UIInputFloat4().Value(q).ReadOnly(True),
                R.UIButton().Label("Copy Pose").Callback(self.copy_pose),
                R.UICheckbox().Label("Show Collision").Bind(self, "show_collision"),
            )

            if actor.classname in ["Actor", "Link"]:
                self.ui_window.append(
                    R.UIDisplayText().Text("Mass"),
                    R.UIInputFloat().Value(actor.mass).ReadOnly(True),
                )

            # collision shapes
            shape_section = R.UISection().Label("Collision Shapes")
            self.ui_window.append(shape_section)
            shapes = actor.get_collision_shapes()
            for shape_idx, shape in enumerate(shapes):
                c0, c1, c2, c3 = shape.get_collision_groups()
                shape_pose = shape.get_local_pose()
                mat = shape.get_physical_material()

                shape_info = (
                    R.UITreeNode()
                    .Label(shape.type)
                    .Id("shape{}".format(shape_idx))
                    .append(
                        R.UIDisplayText().Text(
                            "Contact offset: {:.3g}".format(shape.contact_offset)
                        ),
                        R.UIDisplayText().Text(
                            "Rest offset: {:.3g}".format(shape.rest_offset)
                        ),
                        R.UIDisplayText().Text(
                            "Patch radius: {:.3g}".format(shape.patch_radius)
                        ),
                        R.UIDisplayText().Text(
                            "Min path radius: {:.3g}".format(shape.min_patch_radius)
                        ),
                        R.UICheckbox().Label("Is trigger").Checked(shape.is_trigger),
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
                        R.UIDisplayText().Text("Local position XYZ"),
                        R.UIInputFloat3()
                        .Id("shape_pos{}".format(shape_idx))
                        .Value(shape_pose.p)
                        .ReadOnly(True),
                        R.UIDisplayText().Text("Local rotation WXYZ"),
                        R.UIInputFloat4()
                        .Id("shape_rot{}".format(shape_idx))
                        .Value(shape_pose.q)
                        .ReadOnly(True),
                    )
                )

                shape_section.append(shape_info)

                if shape.type == "sphere":
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Sphere radius: {:.3g}".format(shape.geometry.radius)
                        )
                    )
                elif shape.type == "capsule":
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Capsule radius: {:.3g}".format(shape.geometry.radius)
                        ),
                        R.UIDisplayText().Text(
                            "Capsule half length: {:.3g}".format(
                                shape.geometry.half_length
                            )
                        ),
                    )
                elif shape.type == "box":
                    x, y, z = shape.geometry.half_size
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Box half size: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                elif shape.type == "convex_mesh":
                    x, y, z = shape.geometry.scale
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Mesh scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                elif shape.type == "nonconvex_mesh":
                    x, y, z = shape.geometry.scale
                    shape_info.append(
                        R.UIDisplayText().Text(
                            "Mesh scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )

            # render shapes
            body_section = R.UISection().Label("Visual Bodies")
            self.ui_window.append(body_section)
            bodies = actor.get_visual_bodies()
            for body_idx, body in enumerate(bodies):
                body_info = (
                    R.UITreeNode().Label(body.type).Id("visual{}".format(body_idx))
                )
                body_section.append(body_info)

                if body.type == "sphere":
                    body_info.append(
                        R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius))
                    )
                elif body.type == "capsule":
                    body_info.append(
                        R.UIDisplayText().Text("Radius: {:.3g}".format(body.radius)),
                        R.UIDisplayText().Text(
                            "Half length: {:.3g}".format(body.half_length)
                        ),
                    )
                elif body.type == "box":
                    x, y, z = body.half_size
                    body_info.append(
                        R.UIDisplayText().Text(
                            "Half size: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                elif body.type == "mesh":
                    x, y, z = body.scale
                    body_info.append(
                        R.UIDisplayText().Text(
                            "Scale: {:.3g} {:.3g} {:.3g}".format(x, y, z)
                        )
                    )
                body_info.append(
                    R.UIDisplayText().Text("Visual id: {}".format(body.visual_id))
                )
                shapes = body.get_render_shapes()
                for shape_idx, shape in enumerate(shapes):
                    mat = shape.material
                    dtex = (
                        (mat.diffuse_texture_filename or "(has texture)")
                        if mat.diffuse_texture
                        else "(no texture)"
                    )
                    rtex = (
                        (mat.roughness_texture_filename or "(has texture)")
                        if mat.roughness_texture
                        else "(no texture)"
                    )
                    mtex = (
                        (mat.metallic_texture_filename or "(has texture)")
                        if mat.metallic_texture
                        else "(no texture)"
                    )
                    ntex = (
                        (mat.normal_texture_filename or "(has texture)")
                        if mat.normal_texture
                        else "(no texture)"
                    )
                    etex = (
                        (mat.emission_texture_filename or "(has texture)")
                        if mat.emission_texture
                        else "(no texture)"
                    )

                    body_info.append(
                        R.UITreeNode()
                        .Label("material {}##{}".format(shape_idx, body_idx))
                        .append(
                            R.UIInputFloat4()
                            .Label("Diffuse")
                            .ReadOnly(True)
                            .Value(mat.base_color),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(dtex)
                            .Label("##dtex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat4()
                            .Label("Emission")
                            .ReadOnly(True)
                            .Value(mat.emission),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(etex)
                            .Label("##etex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat()
                            .Label("Roughness")
                            .ReadOnly(True)
                            .Value(mat.roughness),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(rtex)
                            .Label("##rtex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat()
                            .Label("Metallic")
                            .ReadOnly(True)
                            .Value(mat.metallic),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(mtex)
                            .Label("##mtex{}_{}".format(shape_idx, body_idx)),
                            R.UIInputFloat()
                            .Label("Specular")
                            .ReadOnly(True)
                            .Value(mat.specular),
                            R.UIDisplayText().Text("Normal map"),
                            R.UIInputText()
                            .ReadOnly(True)
                            .Value(ntex)
                            .Label("##ntex{}_{}".format(shape_idx, body_idx)),
                        )
                    )

        if isinstance(self.selected_entity, sapien.LightEntity):
            light = self.selected_entity
            p = light.pose.p
            q = light.pose.q
            self.ui_window.append(
                R.UIDisplayText().Text("Name: {}".format(light.name)),
                R.UIDisplayText().Text("Class: {}".format(light.classname)),
            )

            def set_shadow(light, enable):
                light.shadow = enable

            self.ui_window.append(
                R.UICheckbox()
                .Label("Shadow")
                .Checked(light.shadow)
                .Callback((lambda light: lambda p: set_shadow(light, p.checked))(light))
            )

            self.ui_window.append(
                R.UIDisplayText().Text("Position XYZ"),
                R.UIInputFloat3()
                .Id("xyz")
                .Value(p)
                .Callback(lambda v: self.set_actor_pose(sapien.Pose(v.value, q))),
                R.UIDisplayText().Text("Rotation WXYZ"),
                R.UIInputFloat4().Value(q).ReadOnly(True),
                R.UIButton().Label("Copy Pose").Callback(self.copy_pose),
            )
            if hasattr(light, "shadow_near"):
                self.ui_window.append(
                    R.UIInputFloat()
                    .Label("Near")
                    .Value(light.shadow_near)
                    .ReadOnly(True),
                    R.UIInputFloat()
                    .Label("Far")
                    .Value(light.shadow_far)
                    .ReadOnly(True),
                )

            if light.classname == "PointLightEntity":
                light: PointLightEntity
                pass
            elif light.classname == "DirectionalLightEntity":
                self.ui_window.append(
                    R.UIInputFloat()
                    .Label("Half Size")
                    .Value(light.shadow_half_size)
                    .ReadOnly(True),
                )
                pass
            elif light.classname == "SpotLightEntity":
                pass

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []
