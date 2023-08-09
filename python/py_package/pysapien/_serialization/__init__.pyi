from __future__ import annotations
import sapien.pysapien._serialization
import typing
import sapien.pysapien
import sapien.pysapien.physx

__all__ = [
    
]


def _serialize_articulation_entity_group(arg0: sapien.pysapien.physx.PhysxArticulation) -> bytes:
    pass
def _serialize_component(arg0: sapien.pysapien.Component) -> bytes:
    pass
def _serialize_entity(arg0: sapien.pysapien.Entity) -> bytes:
    pass
def _serialize_entity_group(arg0: list[sapien.pysapien.Entity]) -> bytes:
    pass
def _serialize_scene(arg0: sapien.pysapien.Scene) -> bytes:
    pass
def _unserialize_component(arg0: bytes) -> sapien.pysapien.Component:
    pass
def _unserialize_entity(arg0: bytes) -> sapien.pysapien.Entity:
    pass
def _unserialize_entity_group(arg0: bytes) -> list[sapien.pysapien.Entity]:
    pass
def _unserialize_scene(arg0: bytes) -> sapien.pysapien.Scene:
    pass
