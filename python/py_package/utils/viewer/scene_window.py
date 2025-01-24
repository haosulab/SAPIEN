#
# Copyright 2025 Hillbot Inc.
# Copyright 2020-2024 UCSD SU Lab
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
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


        if self.ui_window is None:
            self.ui_window = R.UIWindow().Pos(410, 10).Size(400, 400).Label("Scene")

        if self.viewer.selected_entity is not None:
            scene = self.viewer.selected_entity.scene
        elif self.viewer.scene is not None:
            scene = self.viewer.scene
        else:
            return

        self.ui_window.remove_children()
        self.ui_window.append(
            R.UIDisplayText().Text(
                "Selected scene: {}".format(scene.id)
            )
        )
        atree = R.UITreeNode().Label("Entities")
        self.ui_window.append(atree)

        for i, entity in enumerate(scene.entities):
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
