import sapien
from sapien import internal_renderer as R

from .plugin import Plugin


class ContactWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self.ui_window = None
        self.enabled = False

    def notify_scene_change(self):
        if not self.viewer.scenes:
            self.reset()

    def build(self):
        if not self.viewer.scene:
            self.ui_window = None
            return

        scene = self.viewer.scene

        if self.ui_window is None:
            self.ui_window = R.UIWindow().Label("Conacts").Pos(10, 10).Size(400, 400)

        self.ui_window.remove_children()
        self.ui_window.append(R.UICheckbox().Label("Enabled").Bind(self, "enabled"))

        if not self.enabled:
            return

        if scene.physx_system:
            px: sapien.physx.PhysxCpuSystem = scene.physx_system

            collision_section = R.UISection().Expanded(True).Label("Collisions")
            near_collision_section = R.UISection().Expanded(True).Label("Contacts")
            self.ui_window.append(collision_section, near_collision_section)

            collisions = []
            near_collisions = []
            for i, c in enumerate(px.get_contacts()):
                contact = R.UITreeNode().Id(str(i))
                contact.append(
                    R.UIButton()
                    .Label(c.bodies[0].name)
                    .Callback(
                        (lambda e: lambda _: self.viewer.select_entity(e))(
                            c.bodies[0].entity
                        )
                    ),
                    R.UIButton()
                    .Label(c.bodies[1].name)
                    .Callback(
                        (lambda e: lambda _: self.viewer.select_entity(e))(
                            c.bodies[1].entity
                        )
                    ),
                )

                if not any(p.impulse @ p.impulse > 1e-7 for p in c.points):
                    near_collisions.append(contact)
                else:
                    for j, p in enumerate(c.points):
                        if p.impulse @ p.impulse > 1e-7:
                            contact.append(
                                R.UITreeNode()
                                .Label("Point")
                                .Id(str(j))
                                .append(
                                    R.UIInputFloat()
                                    .Label("separation")
                                    .Value(p.separation),
                                    R.UIInputFloat3().Label("Impulse").Value(p.impulse),
                                )
                            )
                    collisions.append(contact)

            if collisions:
                collision_section.append(*collisions)
            if near_collisions:
                near_collision_section.append(*near_collisions)

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []

    def close(self):
        self.reset()
