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
    def scene(self):
        return self.viewer.scene

    @property
    def selected_entity(self):
        return self.viewer.selected_entity

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if self.ui_window is None:
            self.ui_window = (
                R.UIWindow()
                .Pos(410, 10)
                .Size(400, 400)
                .Label("Scene")
                .append(
                    R.UITreeNode()
                    .Label("World")
                    .append(
                        R.UITreeNode().Label("Entities"),
                        # R.UITreeNode().Label("Articulations"),
                        # R.UITreeNode().Label("Lights"),
                    ),
                )
            )

        # atree, arttree, ltree = self.ui_window.get_children()[0].get_children()
        (atree,) = self.ui_window.get_children()[0].get_children()
        atree: R.UITreeNode
        # arttree: R.UITreeNode
        # ltree: R.UITreeNode
        atree.remove_children()
        # arttree.remove_children()
        # ltree.remove_children()

        for i, entity in enumerate(self.scene.entities):
            atree.append(
                R.UISelectable()
                .Label(entity.name if entity.name else "(no name)")
                .Id("entity{}".format(i))
                .Selected(self.selected_entity == entity)
                .Callback((lambda link: lambda _: self.select_entity(link))(entity))
            )

        # for i, art in enumerate(self.scene.get_all_articulations()):
        #     art_node = (
        #         R.UITreeNode()
        #         .Label(art.name if art.name else "(no name)")
        #         .Id("art{}".format(i))
        #     )
        #     for j, link in enumerate(art.get_links()):
        #         art_node.append(
        #             R.UISelectable()
        #             .Label(link.name if link.name else "(no name)")
        #             .Id("link{}".format(j))
        #             .Selected(self.selected_entity == link)
        #             .Callback((lambda link: lambda _: self.select_entity(link))(link))
        #         )
        #     arttree.append(art_node)

        # for i, light in enumerate(self.scene.get_all_lights()):
        #     ltree.append(
        #         R.UISelectable()
        #         .Label(light.name if light.name else "(no name)")
        #         .Id("light{}".format(i))
        #         .Selected(self.selected_entity == light)
        #         .Callback((lambda light: lambda _: self.select_entity(light))(light))
        #     )

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []
