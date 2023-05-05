from ..core import Scene
from .serialization import SerializedScene

from collections import OrderedDict

class KeyFrameData:
    def __init__(self):
        self.keyframes = OrderedDict() # Regular dict preserves order only after Python 3.7

    def insert(self, frame: int, scene: Scene):
        if frame in self.keyframes:
            return # No duplication allowed
        
        self.keyframes[frame] = SerializedScene(scene)
        
        # Sort by keys
        self.keyframes = OrderedDict(sorted(self.keyframes.items(), key=lambda item: item[0]))
    
    def update(self, frame: int, scene: Scene):
        if frame not in self.keyframes:
            return
        
        self.keyframes[frame].update_state_from(scene)

    def delete(self, frame: int):
        if frame not in self.keyframes:
            return
        
        self.keyframes.pop(frame)
