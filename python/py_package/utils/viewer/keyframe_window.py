import os
import platform

import numpy as np
import sapien
from sapien import internal_renderer as R

from .plugin import Plugin
from ..serialization import SerializedScene


class Keyframe(R.UIKeyframe):
    def __init__(self, serialized_scene, frame=0):
        super().__init__()
        self.serialized_scene = serialized_scene
        self._frame = frame

    def frame(self):
        return self._frame

    def set_frame(self, frame):
        self._frame = frame

    def __repr__(self):
        return "Keyframe(scene, frame={})".format(self._frame)


class Duration(R.UIDuration):
    DEFAULT_DEFINITION = """import sapien
import numpy as np


class Reward:
    def __init__(self, env, scene: sapien.Scene):
        self.env = env
        self.scene = scene

    def compute(self):
        return 0
"""

    def __init__(self, keyframe0, keyframe1, name="", definition=DEFAULT_DEFINITION):
        super().__init__()
        self._keyframe0 = keyframe0
        self._keyframe1 = keyframe1
        self._name = name
        self.definition = definition

    def keyframe0(self):
        return self._keyframe0

    def keyframe1(self):
        return self._keyframe1

    def set_name(self, name):
        self._name = name

    def name(self):
        return self._name


def serialize_keyframes(keyframes, durations):
    s_keyframes = [(f.serialized_scene, f._frame) for f in keyframes]
    f2i = dict((f, i) for i, f in enumerate(keyframes))
    s_durations = [
        (f2i[d._keyframe0], f2i[d._keyframe1], d._name, d.definition) for d in durations
    ]

    return s_keyframes, s_durations


def deserialize_keyframes(state):
    s_keyframes, s_durations = state

    keyframes = [Keyframe(*f) for f in s_keyframes]
    durations = [
        Duration(keyframes[d[0]], keyframes[d[1]], d[2], d[3]) for d in s_durations
    ]
    return keyframes, durations


class KeyframeWindow(Plugin):
    def __init__(self):
        self.reset()

    @property
    def scene(self):
        return self.viewer.scene

    @property
    def keyframes(self):
        return self.keyframe_editor.get_keyframes()

    def reset(self):
        self.ui_window = None
        self.popup = None
        self.show_popup = False
        self.keyframe_editor = None
        self.key_frame_scenes = []
        self.edited_duration = None
        self._editor_file_name = None

        self.current_frame = 0
        self.total_frames = 32

    def close(self):
        self.reset()

    def close_popup(self):
        self.show_popup = False
        if self._editor_file_name:
            try:
                os.remove(self._editor_file_name)
            except FileNotFoundError:
                pass

        self._editor_file_name = None

    def open_popup(self):
        self.show_popup = True
        self.popup.get_children()[1].Value(self.edited_duration.name())
        self.popup.get_children()[2].Value(self.edited_duration.definition)

    def duration_name_change(self, text):
        self.edited_duration.set_name(text.value)

    def duration_definition_change(self, text):
        self.edited_duration.definition = text.value

    def confirm_popup(self, _):
        self.edited_duration.set_name(self.popup.get_children()[1].value)
        self.edited_duration.definition = self.popup.get_children()[2].value
        self.close_popup()

    def cancel_popup(self, _):
        self.close_popup()

    def open_in_editor(self, _):
        if self._editor_file_name is None:
            import tempfile

            editor_file = tempfile.NamedTemporaryFile(
                mode="w+",
                prefix=self.edited_duration.name(),
                suffix=".py",
                delete=False,
            )
            editor_file.write(self.edited_duration.definition)
            editor_file.close()
            self._editor_file_name = editor_file.name

        if platform.system() == "Linux":
            os.system("gedit '{}' & disown".format(self._editor_file_name))
        elif platform.system() == "Windows":
            print(
                "Please open and edit the file {} manually".format(
                    self._editor_file_name
                )
            )

    def notify_window_focus_change(self, focused):
        if focused and self.edited_duration and self._editor_file_name:
            with open(self._editor_file_name, "r") as f:
                self.edited_duration.definition = f.read()
                self.popup.get_children()[2].Value(self.edited_duration.definition)

    def build(self):
        if self.scene is None:
            self.ui_window = None
            return

        if self.ui_window:
            return

        self.popup = (
            R.UIPopup()
            .Label("Edit Duration")
            .EscCallback(self.close_popup)
            .append(
                R.UISameLine().append(
                    R.UIButton().Label("Confirm").Callback(self.confirm_popup),
                    R.UIButton().Label("Cancel").Callback(self.cancel_popup),
                    R.UIButton().Label("Open in Editor").Callback(self.open_in_editor),
                ),
                R.UIInputText().Label("Name").Callback(self.duration_name_change),
                R.UIInputTextMultiline()
                .Label("Definition")
                .Callback(self.duration_definition_change),
            )
        )

        self.keyframe_editor = (
            R.UIKeyframeEditor(self.viewer.window.get_content_scale())
            .BindCurrentFrame(self, "current_frame")
            .BindTotalFrames(self, "total_frames")
            .AddKeyframeCallback(self.add_keyframe)
            .MoveKeyframeCallback(self.move_keyframe)
            .AddDurationCallback(self.add_duration)
            .DoubleClickKeyframeCallback(self.load_keyframe)
            .DoubleClickDurationCallback(self.edit_duration)
            .append(
                R.UIButton().Label("Export").Callback(self.editor_export),
                R.UIButton().Label("Import").Callback(self.editor_import),
            )
        )

        self.ui_window = (
            R.UIWindow().Label("Key Frame Editor").append(self.keyframe_editor)
        )

    def add_keyframe(self, frame):
        frame = Keyframe(SerializedScene(self.scene), frame)
        self.keyframe_editor.add_keyframe(frame)
        print(self.keyframes)

    def add_duration(self, frame0, frame1):
        duration = Duration(frame0, frame1, "New Reward")
        self.keyframe_editor.add_duration(duration)

    def move_keyframe(self, frame, time):
        frame.set_frame(time)

    def load_keyframe(self, frame):
        s_scene = frame.serialized_scene
        s_scene.dump_state_into(self.scene)

    def edit_duration(self, duration):
        self.edited_duration = duration
        self.open_popup()

    def get_ui_windows(self):
        self.build()
        windows = []
        if self.ui_window:
            windows.append(self.ui_window)
        if self.show_popup:
            windows.append(self.popup)
        return windows

    def get_editor_state(self):
        keyframes = self.keyframe_editor.get_keyframes()
        durations = self.keyframe_editor.get_durations()
        return (
            self.total_frames,
            self.current_frame,
            serialize_keyframes(keyframes, durations),
        )

    def set_editor_state(self, state):
        self.total_frames, self.current_frame, s = state
        keyframes, durations = deserialize_keyframes(s)
        self.keyframe_editor.set_state(keyframes, durations)

    def editor_export(self, _):
        self._state = self.get_editor_state()

    def editor_import(self, _):
        self.set_editor_state(self._state)
