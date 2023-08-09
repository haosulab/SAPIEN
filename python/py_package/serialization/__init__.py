from ..pysapien._serialization import _serialize_scene, _unserialize_scene
from ..pysapien import Component
import pickle
from ..wrapper.scene import Scene
from typing import List


def _serialize_python_components(components: List[Component]):
    data = []
    for c in components:
        data.append((c.__class__, c.save(), c._serialization_id))
    return pickle.dumps(data)


def _unserialize_python_components(data: bytes) -> List[Component]:
    components = []
    for cls, d, sid in pickle.loads(data):
        load = getattr(cls, "load", None)
        if not load:
            raise Exception("load method is missing")

        component = load(d)
        component._serialization_id = sid
        components.append(component)

    return components


# def serialize_scene(scene: Scene):
#     scene_data = pickle.dumps(scene)
#     python_component_data = _serialize_python_components(
#         scene._find_all_python_components()
#     )
#     return pickle.dumps((scene_data, python_component_data))


# def unserialize_scene(data: bytes):
#     scene_data, python_component_data = pickle.loads(data)
#     scene = pickle.loads(scene_data)
#     components = _unserialize_python_components(python_component_data)
#     scene._swap_in_python_components(components)

#     return scene
