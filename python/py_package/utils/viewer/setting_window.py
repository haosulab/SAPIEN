import os

from sapien import internal_renderer as R

from .plugin import Plugin


class SettingWindow(Plugin):
    def __init__(self):
        self.reset()

    def reset(self):
        self.ui_window = None

    def notify_scene_change(self):
        if not self.viewer.scenes:
            self.reset()

    def build(self):
        if not self.viewer.scene:
            self.ui_window = None
            return

        scene = self.viewer.scene

        if self.ui_window is None:
            self.ui_window = R.UIWindow().Label("Settings").Pos(10, 10).Size(400, 400)

        self.ui_window.remove_children()
        if scene.physx_system:
            px = scene.physx_system

            self.ui_window.append(
                R.UISection()
                .Expanded(True)
                .Label("PhysX Settings")
                .append(
                    R.UIInputFloat()
                    .ReadOnly(True)
                    .Label("Time Step")
                    .Value(px.timestep),
                    R.UIInputFloat3()
                    .ReadOnly(True)
                    .Label("Gravity")
                    .Value(px.config.gravity),
                    R.UICheckbox().Label("TGS").Checked(px.config.enable_tgs),
                    R.UICheckbox().Label("PCM").Checked(px.config.enable_pcm),
                    R.UIInputFloat()
                    .ReadOnly(True)
                    .Label("Bounce Threshold")
                    .Value(px.config.bounce_threshold),
                )
            )

    def get_ui_windows(self):
        self.build()
        if self.ui_window:
            return [self.ui_window]
        return []

    def close(self):
        self.reset()
