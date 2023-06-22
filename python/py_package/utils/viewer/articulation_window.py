from .plugin import Plugin
from sapien.core import renderer as R
import sapien.core as sapien


class ArticulationWindow(Plugin):
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

    def notify_selected_entity_change(self):
        if isinstance(self.selected_entity, sapien.LinkBase):
            self.articulation: sapien.ArticulationBase = (
                self.selected_entity.get_articulation()
            )
            self.joint_details = [False] * self.articulation.dof

    def set_joint_details(self, index, v):
        self.joint_details[index] = v

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        self.ui_window = R.UIWindow().Label("Articulation")
        if (
            not self.selected_entity
            or not isinstance(self.selected_entity, sapien.ActorBase)
            or self.selected_entity.classname not in ["Link", "KinematicLink"]
        ):
            self.ui_window.append(R.UIDisplayText().Text("No articulation selected."))
            return

        art = self.selected_entity.get_articulation()
        art: ArticulationBase
        self.ui_window.append(
            R.UIDisplayText().Text(
                "Name: {}".format(art.name if art.name else "(no name)")
            ),
            R.UIDisplayText().Text("Class: {}".format(art.classname)),
            R.UIDisplayText().Text("Base Link Id: {}".format(art.get_links()[0].id)),
        )
        uijoints = R.UISection().Label("Joints")
        joints = []
        for j in art.get_joints():
            if j.get_dof() > 0:
                joints.append(j)

        def wrapper(art, i, qpos):
            def callback(slider):
                qpos[i] = slider.value
                art.set_qpos(qpos)

            return callback

        qpos = art.get_qpos()
        for i, (q, j) in enumerate(zip(qpos, joints)):
            line = R.UISameLine()
            uijoints.append(line)
            line.append(
                R.UISliderFloat()
                .WidthRatio(0.5)
                .Id("joint_{}".format(i))
                .Min(max(j.get_limits()[0][0], -20))
                .Max(min(j.get_limits()[0][1], 20))
                .Value(q)
                .Callback(wrapper(art, i, qpos)),
            )
            if art.classname == "Articulation":
                j: Joint

                if self.joint_details[i]:
                    line.append(
                        R.UIButton()
                        .Label("-")
                        .Id(str(i))
                        .Width(40)
                        .Callback(
                            (lambda i: lambda _: self.set_joint_details(i, False))(i)
                        ),
                        R.UIDisplayText().Text(j.name),
                    )

                    uijoints.append(
                        R.UISliderFloat()
                        .Label("Drive Target")
                        .Id(str(i))
                        .WidthRatio(0.5)
                        .Min(max(j.get_limits()[0][0], -20))
                        .Max(min(j.get_limits()[0][1], 20))
                        .Value(j.get_drive_target())
                        .Callback((lambda j: lambda p: j.set_drive_target(p.value))(j)),
                        R.UIInputFloat()
                        .Label("Damping")
                        .Id(str(i))
                        .WidthRatio(0.5)
                        .Value(j.damping)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness,
                                    p.value,
                                    j.force_limit,
                                    j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Stiffness")
                        .Id(str(i))
                        .WidthRatio(0.5)
                        .Value(j.stiffness)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    p.value,
                                    j.damping,
                                    j.force_limit,
                                    j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Force Limit")
                        .Id(str(i))
                        .WidthRatio(0.5)
                        .Value(j.force_limit)
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness,
                                    j.damping,
                                    p.value,
                                    j.drive_mode,
                                )
                            )(j)
                        ),
                        R.UIInputFloat()
                        .Label("Friction")
                        .Id(str(i))
                        .WidthRatio(0.5)
                        .Value(j.friction)
                        .Callback((lambda j: lambda p: j.set_friction(p.value))(j)),
                        R.UICheckbox()
                        .Label("Acceleration")
                        .Id(str(i))
                        .Checked(j.drive_mode == "acceleration")
                        .Callback(
                            (
                                lambda j: lambda p: j.set_drive_property(
                                    j.stiffness,
                                    j.damping,
                                    j.force_limit,
                                    "acceleration" if p.checked else "force",
                                )
                            )(j)
                        ),
                        R.UIDummy().Height(20),
                    )
                else:
                    line.append(
                        R.UIButton()
                        .Label("+")
                        .Id(str(i))
                        .Width(40)
                        .Callback(
                            (lambda i: lambda _: self.set_joint_details(i, True))(i)
                        ),
                        R.UIDisplayText().Text(j.name),
                    )

        self.ui_window.append(uijoints)

        def wrapper(art):
            def copy_to_clipboard(_):
                import pyperclip

                pyperclip.copy(f"[{', '.join([str(x) for x in art.get_qpos()])}]")

            return copy_to_clipboard

        self.ui_window.append(
            R.UIButton().Label("Copy Joint Positions").Callback(wrapper(art))
        )

        def wrapper(art, show):
            def show_link_collision(_):
                for link in art.get_links():
                    link.render_collision(show)

            return show_link_collision

        self.ui_window.append(
            R.UISameLine().append(
                R.UIButton().Label("Show").Callback(wrapper(art, True)),
                R.UIButton().Label("Hide").Callback(wrapper(art, False)),
                R.UIDisplayText().Text("Collision"),
            ),
        )

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []
