from .actor_builder import ActorBuilder
from .. import pysapien as sapien
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class MimicJointRecord:
    joint: str
    mimic: str
    multiplier: float
    offset: float


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
        self.link_builders: List[LinkBuilder] = []
        self.mimic_joint_records: List[MimicJointRecord] = []

    def set_initial_pose(self, pose):
        self.initial_pose = pose

    def set_scene(self, scene: sapien.Scene):
        self.scene = scene
        return self

    def create_link_builder(self, parent: LinkBuilder = None):
        if self.link_builders:
            assert parent and parent in self.link_builders

        builder = LinkBuilder(len(self.link_builders), parent)
        self.link_builders.append(builder)

        return builder

    def build_entities(self, fix_root_link=None):
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

            links.append(link_component)
            entities.append(entity)

        if fix_root_link is not None:
            entities[0].components[0].joint.type = (
                "fixed" if fix_root_link else "undefined"
            )
        entities[0].pose = self.initial_pose
        return entities

    def build(
        self, fix_root_link=None, build_mimic_joints=True
    ) -> sapien.physx.PhysxArticulation:
        assert self.scene is not None
        links = self.build_entities(fix_root_link=fix_root_link)

        articulation: sapien.physx.PhysxArticulation = (
            links[0]
            .find_component_by_type(sapien.physx.PhysxArticulationLinkComponent)
            .articulation
        )

        if build_mimic_joints:
            for mimic in self.mimic_joint_records:
                joint = articulation.find_joint_by_name(mimic.joint)
                mimic_joint = articulation.find_joint_by_name(mimic.mimic)
                multiplier = mimic.multiplier
                offset = mimic.offset

                # joint mimics parent
                if joint.parent_link == mimic_joint.child_link:
                    if joint.parent_link.parent is None:
                        # TODO warn
                        # tendon must be attached to grandparent
                        continue
                    root = joint.parent_link.parent
                    parent = joint.parent_link
                    child = joint.child_link
                    articulation.create_fixed_tendon(
                        [root, parent, child],
                        [0, -multiplier, 1],
                        [0, -1 / multiplier, 1],
                        rest_length=offset,
                        stiffness=1e5,
                    )
                # 2 children mimic each other
                if joint.parent_link == mimic_joint.parent_link:
                    assert joint.parent_link is not None
                    root = joint.parent_link
                    articulation.create_fixed_tendon(
                        [root, joint.child_link, mimic_joint.child_link],
                        [0, -multiplier, 1],
                        [0, -1 / multiplier, 1],
                        rest_length=offset,
                        stiffness=1e5,
                    )

        for l in links:
            self.scene.add_entity(l)
        return articulation
