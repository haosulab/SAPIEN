from ..core import Scene
from .serialization import SerializedScene

from typing import Optional
from copy import deepcopy as copy


class KeyFrame:
    def __init__(self, frame: int, scene: Scene):
        self._frame = frame
        self._serialized_scene = SerializedScene(scene)
        self._container = None
    
    def set_container(self, container: 'KeyFrameContainer'):
        self._container = container

    def clear_container(self):
        self._container = None

    def set_frame(self, frame: int):
        self._frame = frame

        if self._container:
            self._container._remove_duplicate(self)
            self._container._sort()

    def get_frame(self) -> int:
        return self._frame

    def update_state(self, scene: Scene):
        self._serialized_scene.update_state_from(scene)

    def get_serialized_scene(self) -> SerializedScene:
        return copy(self._serialized_scene)


class KeyFrameContainer:
    def __init__(self):
        self.key_frames = []

    def _sort(self):
        self.key_frames = sorted(self.key_frames, key=lambda kf: kf.get_frame())
    
    def _remove_duplicate(self, key_frame: KeyFrame):
        frame_list = [kf.get_frame() for kf in self.key_frames]
        if key_frame.get_frame() not in frame_list:
            return
        
        for i, frame in enumerate(frame_list):
            if frame == key_frame.get_frame() and self.key_frames[i] is not key_frame:
                self.key_frames.pop(i)
                break

    def find(self, frame: int) -> Optional[KeyFrame]:
        frame_list = [kf.get_frame() for kf in self.key_frames]
        if frame not in frame_list:
            return None
        
        return self.key_frames[frame_list.index(frame)]
    
    def find_by_index(self, index: int) -> KeyFrame:
        return self.key_frames[index]

    def insert(self, key_frame: KeyFrame):
        frame_list = [kf.get_frame() for kf in self.key_frames]
        if key_frame.get_frame() in frame_list:
            return # No duplication allowed
        
        self.key_frames.append(key_frame)
        self._sort()

    def delete(self, frame: int):
        frame_list = [kf.get_frame() for kf in self.key_frames]
        if frame not in frame_list:
            return
        
        key_frame = self.key_frames.pop(frame_list.index(frame))
        key_frame.clear_container()
    