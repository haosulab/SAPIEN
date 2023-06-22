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
