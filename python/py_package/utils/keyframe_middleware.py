from ..core import Scene
from .serialization import SerializedScene

class KeyFrameManager:
    """
    This class stores additional key frame information in addition to information stored in C++.
    """
    def __init__(self):
        self.s_scenes = [] # Index corresponds to KeyFrame's id
    
    def insert(self, scene: Scene):
        self.s_scenes.append(SerializedScene(scene))
    
    def delete(self, id: int):
        self.s_scenes[id] = None

    def get_serialized_scene(self, id: int) -> SerializedScene:
        return self.s_scenes[id]
