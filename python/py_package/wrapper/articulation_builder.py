from .actor_builder import ActorBuilder
from .. import pysapien as sapien
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class JointRecord:
    joint_type: str = "undefined"  # "fixed", "prismatic", "revolute"
    limits: Tuple[float] = (-np.inf, np.inf)
    pose_in_parent: sapien.Pose = sapien.Pose()
    pose_in_child: sapien.Pose = sapien.Pose()
    friction: float = 0
    damping: float = 0
    name: str = ""


class LinkBuilder(ActorBuilder):
    def __init__(self, index: int, parent):
        super().__init__()
        self.parent = parent
        self.index = index
        self.joint_record = JointRecord()
        self.physx_body_type = "link"

    def set_joint_name(self, name):
        self.joint_record.name = name

    def set_joint_properties(
        self, type, limits, pose_in_parent, pose_in_child, friction=0, damping=0
    ):
        self.joint_record = JointRecord(
            joint_type=type,
            limits=limits,
            pose_in_parent=pose_in_parent,
            pose_in_child=pose_in_child,
            friction=friction,
            damping=damping,
            name=self.joint_record.name,
        )

    def _check(self):
        if self.parent is None:
            assert self.joint_record.joint_type in ["fixed", "undefined"]
        else:
            assert self.joint_record.joint_type in [
                "fixed",
                "revolute",
                "revolute_unwrapped",
                "prismatic",
                "continuous",
            ]


class ArticulationBuilder:
    def __init__(self):
        self.initial_pose = sapien.Pose()

    def set_scene(self, scene: sapien.Scene):
        self.scene = scene
        self.link_builders: List[LinkBuilder] = []
        return self

    def create_link_builder(self, parent: LinkBuilder = None):
        if self.link_builders:
            assert parent and parent in self.link_builders

        builder = LinkBuilder(len(self.link_builders), parent)
        self.link_builders.append(builder)

        return builder

    def build_entities(self):
        entities = []
        links = []
        for b in self.link_builders:
            b._check()
            b.physx_body_type = "link"

            entity = sapien.Entity()

            link_component = b.build_physx_component(
                links[b.parent.index] if b.parent else None
            )

            entity.add_component(link_component)
            if b.visual_records:
                entity.add_component(b.build_render_component())
            entity.name = b.name

            # if b.parent:
            #     link_component.set_parent(links[b.parent.index])

            link_component.name = b.name
            link_component.joint.name = b.joint_record.name
            link_component.joint.type = b.joint_record.joint_type
            link_component.joint.pose_in_child = b.joint_record.pose_in_child
            link_component.joint.pose_in_parent = b.joint_record.pose_in_parent

            if link_component.joint.type in [
                "revolute",
                "prismatic",
                "revolute_unwrapped",
            ]:
                link_component.joint.limit = np.array(b.joint_record.limits).flatten()
                link_component.joint.set_drive_property(0, b.joint_record.damping)

            if link_component.joint.type == "continuous":
                link_component.joint.limit = [-np.inf, np.inf]
                link_component.joint.set_drive_property(0, b.joint_record.damping)

            links.append(link_component)
            entities.append(entity)

        return entities

    def build(self, fix_root_link=None):
        assert self.scene is not None
        links = self.build_entities()

        if fix_root_link is not None:
            links[0].components[0].joint.type = (
                "fixed" if fix_root_link else "undefined"
            )

        for l in links:
            self.scene.add_entity(l)
        links[0].pose = self.initial_pose
        return l.components[0].articulation
