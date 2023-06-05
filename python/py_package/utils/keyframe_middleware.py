from sapien.core import renderer as R
import copy


class KeyFrameEditorSnapshot:
    def __init__(self, c: R.UIKeyFrameEditor, serialized_scenes: list):
        self.key_frame_id_state = c.get_key_frame_id_generator().__getstate__()
        self.reward_id_state = c.get_reward_id_generator().__getstate__()

        self.serialized_key_frames = []  # Sorted by frame
        for kf in c.get_key_frames_in_used():
            self.serialized_key_frames.append(self.SerializedUIKeyFrame(kf))

        self.serialized_rewards = []  # Same order as in lister
        for reward in c.get_rewards_in_used():
            self.serialized_rewards.append(self.SerializedUIReward(reward))

        self.serialized_scenes = []  # Index corresponds to key frame's id
        for s_scene in serialized_scenes:
            self.serialized_scenes.append(copy.deepcopy(s_scene))

    class SerializedUIKeyFrame:
        def __init__(self, kf: R.UIKeyFrame):
            self.id = kf.get_id()
            self.frame = kf.frame

    class SerializedUIReward:
        def __init__(self, reward: R.UIReward):
            self.id = reward.get_id()
            self.kf1_id = reward.kf1_id
            self.kf2_id = reward.kf2_id
            self.name = reward.name
            self.definition = reward.definition
