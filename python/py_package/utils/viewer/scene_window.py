from .plugin import Plugin
from sapien import internal_renderer as R
import sapien as sapien


class SceneWindow(Plugin):
    def __init__(self):
        self.reset()

    def init(self, viewer):
        super().init(viewer)

    def reset(self):
        self.ui_window = None

    def close(self):
        self.reset()

    def select_entity(self, entity: sapien.Entity):
        self.viewer.select_entity(entity)

    @property
    def selected_entity(self):
        return self.viewer.selected_entity

    def build(self):
        if self.viewer.render_scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.ui_window = R.UIWindow().Pos(410, 10).Size(400, 400).Label("Scene")

        if self.viewer.selected_entity is not None:
            self.ui_window.remove_children()
            self.ui_window.append(
                R.UIDisplayText().Text(
                    "Selected scene: {}".format(self.viewer.selected_entity.scene.id)
                )
            )
            atree = R.UITreeNode().Label("Entities")
            self.ui_window.append(atree)

            for i, entity in enumerate(self.selected_entity.scene.entities):
                atree.append(
                    R.UISelectable()
                    .Label(entity.name if entity.name else "(no name)")
                    .Id("entity{}".format(i))
                    .Selected(self.selected_entity == entity)
                    .Callback((lambda link: lambda _: self.select_entity(link))(entity))
                )

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []
