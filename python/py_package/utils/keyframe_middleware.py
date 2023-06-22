from sapien.core import renderer as R
import copy


class KeyframeEditorSnapshot:
    def __init__(self, c: R.UIKeyframeEditor, serialized_scenes: list):
        self.key_frame_id_state = c.get_key_frame_id_generator_state()
        self.duration_id_state = c.get_duration_id_generator_state()

        self.serialized_key_frames = []  # Sorted by frame
        for kf in c.get_key_frames_in_used():
            self.serialized_key_frames.append(self.SerializedUIKeyframe(kf))

        self.serialized_durations = []  # Same order as in lister
        for duration in c.get_durations_in_used():
            self.serialized_durations.append(self.SerializedUIDuration(duration))

        self.serialized_scenes = []  # Index corresponds to key frame's id
        for s_scene in serialized_scenes:
            self.serialized_scenes.append(copy.deepcopy(s_scene))

    class SerializedUIKeyframe:
        def __init__(self, kf: R.UIKeyframe):
            self.id = kf.get_id()
            self.frame = kf.frame

    class SerializedUIDuration:
        def __init__(self, duration: R.UIDuration):
            self.id = duration.get_id()
            self.kf1_id = duration.kf1_id
            self.kf2_id = duration.kf2_id
            self.name = duration.name
            self.definition = duration.definition
