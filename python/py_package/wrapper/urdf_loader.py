import math
import os
from lxml import etree
from lxml import etree as ET
from pathlib import Path

import numpy as np

from ..pysapien.physx import PhysxArticulation, PhysxMaterial
from ..pysapien.render import RenderCameraComponent, RenderMaterial, RenderTexture2D
from ..pysapien import Pose
from .articulation_builder import ArticulationBuilder, MimicJointRecord
from .urchin import URDF


def _try_very_hard_to_find_file(filename, urdf_dir, package_dir=None):
    urdf_dir = Path(urdf_dir).absolute()
    package_dir = Path(package_dir).absolute() if package_dir is not None else None

    fpath = None
    if filename.startswith("package://"):
        filename = filename[10:]
        if package_dir is not None:
            fpath = package_dir / filename

        parent_dir = urdf_dir
        while True:
            fpath = parent_dir / filename
            if fpath.is_file():
                break
            if parent_dir == Path("/"):
                break
            parent_dir = parent_dir.parent
    else:
        fpath = urdf_dir / filename

    if fpath is None or not fpath.is_file():
        # TODO: warn
        return filename

    return str(fpath)


class URDFLoader:
    def __init__(self):
        self.fix_root_link = True

        self.load_multiple_collisions_from_file = False
        self.multiple_collisions_decomposition = "none"
        self.multiple_collisions_decomposition_params = dict()

        self.collision_is_visual = False
        self.revolute_unwrapped = False
        self.scale = 1.0

        self._material = None
        self._patch_radius = 0
        self._min_patch_radius = 0
        self._density = 1000
        self._link_material = dict()
        self._link_patch_radius = dict()
        self._link_min_patch_radius = dict()
        self._link_density = dict()

    def _get_material(self, link_name, index):
        if link_name in self._link_material:
            return PhysxMaterial(*self._link_material[link_name])
        if self._material is not None:
            return PhysxMaterial(*self._material)
        return None

    def _get_patch_radius(self, link_name, index):
        if link_name in self._link_patch_radius:
            return self._link_patch_radius[link_name]
        return self._patch_radius

    def _get_min_patch_radius(self, link_name, index):
        if link_name in self._link_min_patch_radius:
            return self._link_min_patch_radius[link_name]
        return self._min_patch_radius

    def _get_density(self, link_name, index):
        if link_name in self._link_density:
            return self._link_density[link_name]
        return self._density

    def set_material(self, static_friction, dynamic_friction, restitution):
        self._material = [static_friction, dynamic_friction, restitution]

    def set_patch_radius(self, patch_radius):
        self._patch_radius = patch_radius

    def set_min_patch_radius(self, min_patch_radius):
        self._min_patch_radius = min_patch_radius

    def set_density(self, density):
        self._density = density

    def set_link_material(
        self, link_name, static_friction, dynamic_friction, restitution
    ):
        self._link_material[link_name] = [
            static_friction,
            dynamic_friction,
            restitution,
        ]

    def set_link_patch_radius(self, link_name, patch_radius):
        self._link_patch_radius[link_name] = patch_radius

    def set_link_min_patch_radius(self, link_name, min_patch_radius):
        self._link_min_patch_radius[link_name] = min_patch_radius

    def set_link_density(self, link_name, density):
        self._link_density[link_name] = density

    def set_scene(self, scene):
        self.scene = scene
        return self

    def parse_srdf(self, srdf_string):
        ignore_pairs = []
        root = ET.fromstring(srdf_string.encode("utf-8"))
        for elem in root.findall("disable_collisions"):
            if elem.attrib["reason"].lower() == "default":
                ignore_pairs.append(set((elem.attrib["link1"], elem.attrib["link2"])))
        return ignore_pairs

    @staticmethod
    def _pose_from_origin(origin, scale):
        origin[:3, 3] = origin[:3, 3] * scale
        return Pose(origin)

    def _parse_cameras(self, extra):
        cameras = []

        sensors = []
        for gazebo in extra.findall("gazebo"):
            if "reference" not in gazebo.attrib:
                continue
            for s in gazebo.findall("sensor"):
                if s.find("parent") is None:
                    ET.SubElement(s, "parent", {"link": gazebo.attrib["reference"]})
                sensors.append(s)

        sensors += extra.findall("sensor")

        for sensor in sensors:
            parent = sensor.find("parent")
            if parent is None:
                continue

            camera = sensor.find("camera")
            if camera is None:
                continue

            xyz = "0 0 0"
            rpy = "0 0 0"
            origin = sensor.find("origin")
            if origin is not None:
                xyz = origin.attrib.get("xyz", "0 0 0")
                rpy = origin.attrib.get("rpy", "0 0 0")

            xyz = [float(x) for x in xyz.split()]
            rpy = [float(x) for x in rpy.split()]
            pose = Pose()
            pose.set_p(xyz)
            pose.set_rpy(rpy)

            link_name = parent.attrib["link"]

            image = camera.find("image")
            assert image is not None

            # handle both <image><width></width></image> and <image width=""/>
            width = image.find("width")
            if width is not None:
                width = width.text
            else:
                width = image.attrib.get("width")
            assert width
            width = int(width)

            height = image.find("height")
            if height is not None:
                height = height.text
            else:
                height = image.attrib.get("height")
            assert height
            height = int(height)

            assert width > 0 and height > 0

            # handle both clip and image attribute
            clip = camera.find("clip")
            if clip is not None:
                near = clip.find("near")
                far = clip.find("far")
                near = near.text
                far = far.text
            else:
                near = image.attrib.get("near")
                far = image.attrib.get("far")

            # default
            near = near or "0.01"
            far = far or "100"

            near = float(near)
            far = float(far)

            fovx = camera.find("horizontal_fov")
            if fovx is not None:
                fovx = fovx.text
            else:
                fovx = image.attrib.get("hfov")

            fovy = camera.find("vertical_fov")
            if fovy is not None:
                fovy = fovy.text
            else:
                fovy = image.attrib.get("vfov")
            assert fovx or fovy

            fovx = float(fovx) if fovx else None
            fovy = float(fovy) if fovy else None

            cameras.append(
                {
                    "parent": link_name,
                    "fovx": fovx,
                    "fovy": fovy,
                    "width": width,
                    "height": height,
                    "near": near,
                    "far": far,
                    "pose": pose,
                }
            )

        return cameras

    def _build_link(self, link, link_builder):
        # inertial
        if (
            link.inertial
            and link.inertial.mass != 0
            and not np.array_equal(link.inertial.inertia, np.zeros((3, 3)))
        ):
            t_inertial2link = self._pose_from_origin(link.inertial.origin, self.scale)
            mass = link.inertial.mass
            inertia = link.inertial.inertia

            if np.array_equal(np.diag(np.diag(inertia)), inertia):
                eigs = np.diag(inertia)
                vecs = np.eye(3)
            else:
                eigs, vecs = np.linalg.eigh(inertia)
                if np.linalg.det(vecs) < 0:
                    vecs[:, 2] = -vecs[:, 2]

            assert all([x > 0 for x in eigs]), "invalid moment of inertia"

            t_inertia2inertial = np.eye(4)
            t_inertia2inertial[:3, :3] = vecs
            t_inertia2inertial = Pose(t_inertia2inertial)

            t_inertial2link = t_inertial2link * t_inertia2inertial
            scale3 = self.scale**3
            scale5 = self.scale**5
            link_builder.set_mass_and_inertia(
                mass * scale3, t_inertial2link, scale5 * eigs
            )

        # visual shapes
        for visual in link.visuals:
            material = None
            if visual.material:
                material = RenderMaterial()
                if visual.material.color is not None:
                    material.base_color = visual.material.color
                elif visual.material.texture is not None:
                    material.diffuse_texture = RenderTexture2D(
                        _try_very_hard_to_find_file(
                            visual.material.texture.filename,
                            self.urdf_dir,
                            self.package_dir,
                        )
                    )

            t_visual2link = self._pose_from_origin(visual.origin, self.scale)
            name = visual.name if visual.name else ""
            if visual.geometry.box:
                link_builder.add_box_visual(
                    t_visual2link,
                    visual.geometry.box.size * self.scale / 2.0,
                    material=material,
                    name=name,
                )
            if visual.geometry.sphere:
                link_builder.add_sphere_visual(
                    t_visual2link,
                    visual.geometry.sphere.radius * self.scale,
                    material=material,
                    name=name,
                )
            if visual.geometry.capsule:
                link_builder.add_capsule_visual(
                    t_visual2link * Pose(q=[0.7071068, 0, 0.7071068, 0]),
                    visual.geometry.capsule.radius * self.scale,
                    visual.geometry.capsule.length * self.scale / 2.0,
                    material=material,
                    name=name,
                )
            if visual.geometry.cylinder:
                link_builder.add_cylinder_visual(
                    t_visual2link * Pose(q=[0.7071068, 0, 0.7071068, 0]),
                    visual.geometry.cylinder.radius * self.scale,
                    visual.geometry.cylinder.length * self.scale / 2.0,
                    material=material,
                    name=name,
                )
            if visual.geometry.mesh:
                if visual.geometry.mesh.scale is not None:
                    scale = visual.geometry.mesh.scale
                else:
                    scale = np.ones(3)

                link_builder.add_visual_from_file(
                    _try_very_hard_to_find_file(
                        visual.geometry.mesh.filename,
                        self.urdf_dir,
                        self.package_dir,
                    ),
                    t_visual2link,
                    scale * self.scale,
                    material=material,
                    name=name,
                )

        # collision shapes
        for cid, collision in enumerate(link.collisions):
            t_collision2link = self._pose_from_origin(collision.origin, self.scale)

            material = self._get_material(link.name, cid)
            density = self._get_density(link.name, cid)
            patch_radius = self._get_patch_radius(link.name, cid)
            min_patch_radius = self._get_min_patch_radius(link.name, cid)

            if collision.geometry.box:
                link_builder.add_box_collision(
                    t_collision2link,
                    collision.geometry.box.size * self.scale / 2.0,
                    material=material,
                    density=density,
                    patch_radius=patch_radius,
                    min_patch_radius=min_patch_radius,
                )
                if self.collision_is_visual:
                    link_builder.add_box_visual(
                        t_collision2link,
                        collision.geometry.box.size * self.scale / 2.0,
                    )
            if collision.geometry.sphere:
                link_builder.add_sphere_collision(
                    t_collision2link,
                    collision.geometry.sphere.radius * self.scale,
                    material=material,
                    density=density,
                    patch_radius=patch_radius,
                    min_patch_radius=min_patch_radius,
                )
                if self.collision_is_visual:
                    link_builder.add_sphere_visual(
                        t_collision2link,
                        collision.geometry.sphere.radius * self.scale,
                    )
            if collision.geometry.capsule:
                link_builder.add_capsule_collision(
                    t_collision2link * Pose(q=[0.7071068, 0, 0.7071068, 0]),
                    collision.geometry.capsule.radius * self.scale,
                    collision.geometry.capsule.length * self.scale / 2.0,
                    material=material,
                    density=density,
                    patch_radius=patch_radius,
                    min_patch_radius=min_patch_radius,
                )
                if self.collision_is_visual:
                    link_builder.add_capsule_visual(
                        t_collision2link * Pose(q=[0.7071068, 0, 0.7071068, 0]),
                        collision.geometry.capsule.radius * self.scale,
                        collision.geometry.capsule.length * self.scale / 2.0,
                    )
            if collision.geometry.cylinder:
                link_builder.add_cylinder_collision(
                    t_collision2link * Pose(q=[0.7071068, 0, 0.7071068, 0]),
                    collision.geometry.cylinder.radius * self.scale,
                    collision.geometry.cylinder.length * self.scale / 2.0,
                    material=material,
                    density=density,
                    patch_radius=patch_radius,
                    min_patch_radius=min_patch_radius,
                )
                if self.collision_is_visual:
                    link_builder.add_cylinder_visual(
                        t_collision2link * Pose(q=[0.7071068, 0, 0.7071068, 0]),
                        collision.geometry.cylinder.radius * self.scale,
                        collision.geometry.cylinder.length * self.scale / 2.0,
                    )

            if collision.geometry.mesh:
                if collision.geometry.mesh.scale is not None:
                    scale = collision.geometry.mesh.scale
                else:
                    scale = np.ones(3)

                filename = _try_very_hard_to_find_file(
                    collision.geometry.mesh.filename,
                    self.urdf_dir,
                    self.package_dir,
                )

                if self.load_multiple_collisions_from_file:
                    link_builder.add_multiple_convex_collisions_from_file(
                        filename,
                        t_collision2link,
                        scale * self.scale,
                        material=material,
                        density=density,
                        patch_radius=patch_radius,
                        min_patch_radius=min_patch_radius,
                        decomposition=self.multiple_collisions_decomposition,
                        decomposition_params=self.multiple_collisions_decomposition_params,
                    )
                else:
                    link_builder.add_convex_collision_from_file(
                        filename,
                        t_collision2link,
                        scale * self.scale,
                        material=material,
                        density=density,
                        patch_radius=patch_radius,
                        min_patch_radius=min_patch_radius,
                    )

                if self.collision_is_visual:
                    link_builder.add_visual_from_file(
                        filename,
                        t_collision2link,
                        scale * self.scale,
                    )

    def _parse_articulation(self, root, fix_base):
        builder = self.scene.create_articulation_builder()
        stack = [root]

        link2builder = dict()
        while stack:
            link_name = stack.pop()
            link = self.name2link[link_name]
            joint = self.link2parent_joint[link_name]

            t_joint2parent = (
                self._pose_from_origin(joint.origin, self.scale) if joint else Pose()
            )

            link_builder = builder.create_link_builder(
                link2builder[joint.parent] if joint else None
            )

            link2builder[link_name] = link_builder

            link_builder.set_name(link_name)
            link_builder.set_joint_name(
                self.link2parent_joint[link_name].name
                if self.link2parent_joint[link_name] is not None
                else ""
            )

            self._build_link(link, link_builder)

            if joint is None:
                # FIXME: fix root link
                link_builder.set_joint_properties(
                    "fixed" if fix_base else "undefined", [], Pose(), Pose()
                )
            else:
                friction = 0
                damping = 0
                if joint.dynamics:
                    friction = joint.dynamics.friction
                    damping = joint.dynamics.damping

                axis = joint.axis
                axis_norm = np.linalg.norm(axis)
                if axis_norm < 1e-3:
                    axis = np.array([1, 0, 0])
                else:
                    axis /= axis_norm

                if abs(axis @ [1, 0, 0]) > 0.9:
                    axis1 = np.cross(axis, [0, 0, 1])
                    axis1 /= np.linalg.norm(axis1)
                else:
                    axis1 = np.cross(axis, [1, 0, 0])
                    axis1 /= np.linalg.norm(axis1)
                axis2 = np.cross(axis, axis1)
                t_axis2joint = np.eye(4)
                t_axis2joint[:3, 0] = axis
                t_axis2joint[:3, 1] = axis1
                t_axis2joint[:3, 2] = axis2
                t_axis2joint = Pose(t_axis2joint)
                t_axis2parent = t_joint2parent * t_axis2joint

                if joint.joint_type == "revolute":
                    link_builder.set_joint_properties(
                        "revolute_unwrapped",
                        [[joint.limit.lower, joint.limit.upper]],
                        t_axis2parent,
                        t_axis2joint,
                        friction,
                        damping,
                    )
                elif joint.joint_type == "continuous":
                    link_builder.set_joint_properties(
                        "revolute",
                        [[-np.inf, np.inf]],
                        t_axis2parent,
                        t_axis2joint,
                        friction,
                        damping,
                    )
                elif joint.joint_type == "prismatic":
                    link_builder.set_joint_properties(
                        "prismatic",
                        [
                            [
                                joint.limit.lower * self.scale,
                                joint.limit.upper * self.scale,
                            ]
                        ],
                        t_axis2parent,
                        t_axis2joint,
                        friction,
                        damping,
                    )
                elif joint.joint_type == "fixed":
                    link_builder.set_joint_properties(
                        "fixed",
                        [],
                        t_axis2parent,
                        t_axis2joint,
                        friction,
                        damping,
                    )
                elif joint.joint_type == "floating":
                    raise Exception("URDF parsing error on floating joint")
                elif joint.joint_type == "plannar":
                    raise Exception("URDF planner joint is not supported")

                if joint.mimic is not None:
                    builder.mimic_joint_records.append(
                        MimicJointRecord(
                            joint.name,
                            joint.mimic.joint,
                            joint.mimic.multiplier,
                            joint.mimic.offset,
                        )
                    )

            for j in reversed(self.link2child_joints[link_name]):
                stack.append(j.child)

        # srdf collision filtering
        group_count = 0
        for [l1, l2] in self.ignore_pairs:
            link1 = self.name2link[l1]
            link2 = self.name2link[l2]

            joint1 = self.link2parent_joint[l1]
            joint2 = self.link2parent_joint[l2]
            p1 = joint1.parent if joint1 else None
            p2 = joint2.parent if joint2 else None

            if p1 == l2 or p2 == l1:
                continue  # parent-child collision is already ignored

            group_count += 1
            if group_count == 32:
                raise Exception(
                    "Too many collision groups. Please use a simpler SRDF file"
                )

            link2builder[l1].collision_groups[2] |= 1 << (group_count - 1)
            link2builder[l2].collision_groups[2] |= 1 << (group_count - 1)

        return builder

    def _parse_actor(self, link_name):
        builder = self.scene.create_actor_builder()
        builder.set_name(link_name)
        self._build_link(self.name2link[link_name], builder)

        joint = self.link2parent_joint[link_name]
        if joint is not None and joint.joint_type == "floating":
            t_joint2parent = (
                self._pose_from_origin(joint.origin, self.scale) if joint else Pose()
            )
            builder.initial_pose = t_joint2parent

        return builder

    def _parse_urdf(self, urdf_string):
        xml = ET.fromstring(urdf_string.encode("utf-8"))

        robot = URDF._from_xml(xml, self.urdf_dir, lazy_load_meshes=True)
        links = robot.links
        joints = robot.joints

        self.name2link = dict(zip([link.name for link in links], links))

        self.link2child_joints = dict()
        for l in links:
            self.link2child_joints[l.name] = []
        for joint in joints:
            self.link2child_joints[joint.parent].append(joint)

        self.link2parent_joint = dict()
        for joint in joints:
            self.link2parent_joint[joint.child] = joint
        self.link2parent_joint[robot.base_link.name] = None

        roots = [robot.base_link.name]
        for joint in joints:
            if joint.joint_type == "floating":
                if joint.parent != robot.base_link.name:
                    raise Exception(
                        "Current URDF loader only supports floating joints as the child of the root link"
                    )
                roots.append(joint.child)
                self.link2child_joints[joint.parent] = [
                    j for j in self.link2child_joints[joint.parent] if j != joint
                ]

        actor_builders = []
        articulation_builders = []
        for root in roots:
            if len(self.link2child_joints[root]) == 0:
                actor_builders.append(self._parse_actor(root))
            else:
                if root == robot.base_link.name:
                    fix_base = self.fix_root_link
                else:
                    fix_base = False

                articulation_builders.append(self._parse_articulation(root, fix_base))

        extra = ET.fromstring(robot.other_xml)
        cameras = self._parse_cameras(extra)

        return articulation_builders, actor_builders, cameras

    def parse(self, urdf_file, srdf_file=None, package_dir=None):
        self.package_dir = package_dir
        self.urdf_dir = os.path.dirname(urdf_file)

        with open(urdf_file, "r") as f:
            urdf_string = f.read()

        if srdf_file is None:
            srdf_file = urdf_file[:-4] + "srdf"
        if os.path.isfile(srdf_file):
            with open(srdf_file, "r") as f:
                self.ignore_pairs = self.parse_srdf(f.read())
        else:
            self.ignore_pairs = []

        return self._parse_urdf(urdf_string)

    def load_multiple(self, urdf_file: str, srdf_file=None, package_dir=None):
        """
        Args:
            urdf_file: filename for URDL file
            srdf_file: SRDF for urdf_file. If srdf_file is None, it defaults to the ".srdf" file with the same as the urdf file
            package_dir: base directory used to resolve asset files in the URDF file. If an asset path starts with "package://", "package://" is simply removed from the file name
        Returns:
            returns Tuple[List[Articulation], List[Entity]]. The first element is the list of loaded multi-body articulations, the second element is the list of loaded single-body entities
        """
        articulation_builders, actor_builders, cameras = self.parse(
            urdf_file, srdf_file, package_dir
        )

        articulations = []
        for b in articulation_builders:
            articulations.append(b.build())

        actors = []
        for b in actor_builders:
            actors.append(b.build())

        name2entity = dict()
        for a in articulations:
            for l in a.links:
                name2entity[l.name] = l.entity

        for a in actors:
            name2entity[a.name] = a

        for cam in cameras:
            cam_component = RenderCameraComponent(cam["width"], cam["height"])
            if cam["fovx"] is not None and cam["fovy"] is not None:
                cam_component.set_fovx(cam["fovx"], False)
                cam_component.set_fovy(cam["fovy"], False)
            elif cam["fovy"] is None:
                cam_component.set_fovx(cam["fovx"], True)
            elif cam["fovx"] is None:
                cam_component.set_fovy(cam["fovy"], True)

            cam_component.near = cam["near"]
            cam_component.far = cam["far"]
            cam_component.local_pose = cam["pose"]
            name2entity[cam["parent"]].add_component(cam_component)

        return articulations, actors

    def load(
        self, urdf_file: str, srdf_file=None, package_dir=None
    ) -> PhysxArticulation:
        """
        Args:
            urdf_file: filename for URDL file
            srdf_file: SRDF for urdf_file. If srdf_file is None, it defaults to the ".srdf" file with the same as the urdf file
            package_dir: base directory used to resolve asset files in the URDF file. If an asset path starts with "package://", "package://" is simply removed from the file name
        Returns:
            returns a single Articulation loaded from the URDF file. It throws an error if multiple objects exists
        """

        articulation_builders, actor_builders, cameras = self.parse(
            urdf_file, srdf_file, package_dir
        )

        if len(articulation_builders) > 1 or len(actor_builders) != 0:
            raise Exception(
                "URDF contains multiple objects, call load_multiple instead"
            )

        articulations = []
        for b in articulation_builders:
            articulations.append(b.build())

        actors = []
        for b in actor_builders:
            actors.append(b.build())

        name2entity = dict()
        for a in articulations:
            for l in a.links:
                name2entity[l.name] = l.entity

        for a in actors:
            name2entity[a.name] = a

        for cam in cameras:
            cam_component = RenderCameraComponent(cam["width"], cam["height"])
            if cam["fovx"] is not None and cam["fovy"] is not None:
                cam_component.set_fovx(cam["fovx"], False)
                cam_component.set_fovy(cam["fovy"], False)
            elif cam["fovy"] is None:
                cam_component.set_fovx(cam["fovx"], True)
            elif cam["fovx"] is None:
                cam_component.set_fovy(cam["fovy"], True)

            cam_component.near = cam["near"]
            cam_component.far = cam["far"]
            cam_component.local_pose = cam["pose"]
            name2entity[cam["parent"]].add_component(cam_component)

        return articulations[0]

    def load_file_as_articulation_builder(
        self, urdf_file, srdf_file=None, package_dir=None
    ) -> ArticulationBuilder:
        articulation_builders, actor_builders, cameras = self.parse(
            urdf_file, srdf_file, package_dir
        )

        if len(articulation_builders) > 1 or len(actor_builders) != 0:
            raise Exception("URDF contains multiple objects")

        return articulation_builders[0]
