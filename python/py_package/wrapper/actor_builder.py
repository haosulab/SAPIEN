from .. import pysapien as sapien
from dataclasses import dataclass
import numpy as np
import typing
import os
from typing import List

from .coacd import do_coacd


@dataclass
class PhysicalMaterialRecord:
    static_friction: float = 0.3
    dynamic_friction: float = 0.3
    restitution: float = 0.0


# TODO: support texture
@dataclass
class VisualMaterialRecord:
    emission: tuple = (0, 0, 0, 1)
    base_color: tuple = (1, 1, 1, 1)
    specular: float = 0
    roughness: float = 1
    metallic: float = 0
    transmission: float = 0
    ior: float = 1.45
    transmission_roughness = 0.0


@dataclass
class CollisionShapeRecord:
    type: str  # one of "convex_mesh", "multiple_convex_meshes", "nonconvex_mesh", "box", "capsule", "sphere", "cylinder"

    # for mesh type
    filename: str = ""

    # mesh & box
    scale: tuple = (1, 1, 1)

    # circle & capsule
    radius: float = 1
    # capsule
    length: float = 1

    material: PhysicalMaterialRecord = None
    pose: sapien.Pose = sapien.Pose()

    density: float = 1000
    patch_radius: float = 0
    min_patch_radius: float = 0
    is_trigger: bool = False

    decomposition: str = "none"
    decomposition_params: dict = None


@dataclass
class MeshRecord:
    vertices: List[List[float]]
    indices: List[List[int]]


@dataclass
class VisualShapeRecord:
    type: str  # one of "file", "box", "capsule", "sphere", "mesh", "cylinder"

    filename: str = ""
    scale: tuple = (1, 1, 1)

    radius: float = 1
    length: float = 1

    mesh: MeshRecord = None
    material: VisualMaterialRecord = None

    pose: sapien.Pose = sapien.Pose()
    name: str = ""


Vec3 = tuple


class ActorBuilder:
    def __init__(self):
        self.collision_records: typing.List[CollisionShapeRecord] = []
        self.visual_records: typing.List[VisualShapeRecord] = []
        self.use_density = True
        self.collision_groups = [1, 1, 0, 0]
        self.scene = None
        self.physx_body_type = "dynamic"
        self.name = ""

        self._mass = 0
        self._cmass_local_pose = sapien.Pose()
        self._inertia = np.zeros(3)
        self._auto_inertial = True

        self.initial_pose = sapien.Pose()

    def set_mass_and_inertia(self, mass, cmass_local_pose, inertia):
        self._mass = mass
        self._cmass_local_pose = cmass_local_pose
        self._inertia = inertia
        self._auto_inertial = False

    def set_name(self, name):
        self.name = name

    def set_scene(self, scene: sapien.Scene):
        self.scene = scene
        return self

    def set_physx_body_type(self, type):
        assert type in ["dynamic", "static", "kinematic"]
        self.physx_body_type = type
        return self

    def build_render_component(self):
        # TODO: handle None material
        record2mat = dict()
        for r in self.visual_records:
            mat = sapien.render.RenderMaterial()

            # TODO: support global default material
            m = r.material
            if m is not None:
                mat.emission = m.emission
                mat.base_color = m.base_color
                mat.specular = m.specular
                mat.roughness = m.roughness
                mat.metallic = m.metallic
                mat.transmission = m.transmission
                mat.ior = m.ior
                mat.transmission_roughness = m.transmission_roughness

            record2mat[id(m)] = mat

        component = sapien.render.RenderBodyComponent()
        component.name = r.name
        for r in self.visual_records:
            if r.type == "plane":
                shape = sapien.render.RenderShapePlane(
                    r.scale, record2mat[id(r.material)]
                )
            elif r.type == "box":
                shape = sapien.render.RenderShapeBox(
                    r.scale, record2mat[id(r.material)]
                )
            elif r.type == "sphere":
                shape = sapien.render.RenderShapeSphere(
                    r.radius, record2mat[id(r.material)]
                )
            elif r.type == "capsule":
                shape = sapien.render.RenderShapeCapsule(
                    r.radius, r.length, record2mat[id(r.material)]
                )
            elif r.type == "cylinder":
                shape = sapien.render.RenderShapeCylinder(
                    r.radius, r.length, record2mat[id(r.material)]
                )
            elif r.type == "file":
                shape = sapien.render.RenderShapeTriangleMesh(
                    r.filename,
                    r.scale,
                    record2mat[id(r.material)] if r.material is not None else None,
                )
            elif r.type == "mesh":
                raise Exception("TODO: Mesh in not implemented yet")
            else:
                raise Exception(f"invalid visual shape type [{r.type}]")

            shape.local_pose = r.pose
            component.attach(shape)
        return component

    def build_physx_component(self, link_parent=None):
        record2mat = dict()
        for r in self.collision_records:
            m = r.material
            if m is not None:
                mat = sapien.physx.PhysxMaterial(
                    m.static_friction, m.dynamic_friction, m.restitution
                )
            else:
                mat = sapien.physx.get_default_material()

            record2mat[id(m)] = mat

        if self.physx_body_type == "dynamic":
            component = sapien.physx.PhysxRigidDynamicComponent()
        elif self.physx_body_type == "kinematic":
            component = sapien.physx.PhysxRigidDynamicComponent()
            component.kinematic = True
        elif self.physx_body_type == "static":
            component = sapien.physx.PhysxRigidStaticComponent()
        elif self.physx_body_type == "link":
            component = sapien.physx.PhysxArticulationLinkComponent(link_parent)
        else:
            raise Exception(f"invalid physx body type [{self.physx_body_type}]")

        for r in self.collision_records:
            if r.type == "plane":
                shape = sapien.physx.PhysxCollisionShapePlane(
                    material=record2mat[id(r.material)],
                )
                shapes = [shape]
            elif r.type == "box":
                shape = sapien.physx.PhysxCollisionShapeBox(
                    half_size=r.scale, material=record2mat[id(r.material)]
                )
                shapes = [shape]
            elif r.type == "capsule":
                shape = sapien.physx.PhysxCollisionShapeCapsule(
                    radius=r.radius,
                    half_length=r.length,
                    material=record2mat[id(r.material)],
                )
                shapes = [shape]
            elif r.type == "cylinder":
                shape = sapien.physx.PhysxCollisionShapeCylinder(
                    radius=r.radius,
                    half_length=r.length,
                    material=record2mat[id(r.material)],
                )
                shapes = [shape]
            elif r.type == "sphere":
                shape = sapien.physx.PhysxCollisionShapeSphere(
                    radius=r.radius,
                    material=record2mat[id(r.material)],
                )
                shapes = [shape]
            elif r.type == "convex_mesh":
                shape = sapien.physx.PhysxCollisionShapeConvexMesh(
                    filename=r.filename,
                    scale=r.scale,
                    material=record2mat[id(r.material)],
                )
                shapes = [shape]
            elif r.type == "nonconvex_mesh":
                shape = sapien.physx.PhysxCollisionShapeTriangleMesh(
                    filename=r.filename,
                    scale=r.scale,
                    material=record2mat[id(r.material)],
                )
                shapes = [shape]
            elif r.type == "multiple_convex_meshes":
                if r.decomposition == "coacd":
                    params = r.decomposition_params
                    if params is None:
                        params = dict()

                    filename = do_coacd(r.filename, **params)
                else:
                    filename = r.filename

                shapes = sapien.physx.PhysxCollisionShapeConvexMesh.load_multiple(
                    filename=filename,
                    scale=r.scale,
                    material=record2mat[id(r.material)],
                )

            else:
                raise Exception(f"invalid collision shape type [{r.type}]")

            for shape in shapes:
                shape.local_pose = r.pose
                shape.set_collision_groups(self.collision_groups)
                shape.set_patch_radius(r.patch_radius)
                shape.set_min_patch_radius(r.min_patch_radius)
                component.attach(shape)

        if not self._auto_inertial and self.physx_body_type != "kinematic":
            component.mass = self._mass
            component.cmass_local_pose = self._cmass_local_pose
            component.inertia = self._inertia

        return component

    def build_entity(self):
        entity = sapien.Entity()
        if self.visual_records:
            entity.add_component(self.build_render_component())
        if self.collision_records:
            entity.add_component(self.build_physx_component())
        entity.name = self.name
        return entity

    def build(self, name=None):
        if name is not None:
            self.set_name(name)
        if self.scene is None:
            raise Exception(
                "you need to set the scene of the actor builder by calling the set_scene method"
            )

        entity = self.build_entity()
        self.scene.add_entity(entity)
        entity.name = self.name
        entity.pose = self.initial_pose
        return entity

    def build_kinematic(self, name=""):
        self.set_physx_body_type("kinematic")
        return self.build(name=name)

    def build_static(self, name=""):
        self.set_physx_body_type("static")
        return self.build(name=name)

    def add_plane_collision(
        self,
        pose: sapien.Pose = sapien.Pose(),
        material: PhysicalMaterialRecord = None,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="plane",
                pose=pose,
                material=material,
                density=0,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_box_collision(
        self,
        pose: sapien.Pose = sapien.Pose(),
        half_size: Vec3 = [1, 1, 1],
        material: PhysicalMaterialRecord = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="box",
                pose=pose,
                scale=half_size,
                material=material,
                density=density,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_capsule_collision(
        self,
        pose: sapien.Pose = sapien.Pose(),
        radius: float = 1,
        half_length: float = 1,
        material: PhysicalMaterialRecord = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="capsule",
                pose=pose,
                radius=radius,
                length=half_length,
                material=material,
                density=density,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_cylinder_collision(
        self,
        pose: sapien.Pose = sapien.Pose(),
        radius: float = 1,
        half_length: float = 1,
        material: PhysicalMaterialRecord = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="cylinder",
                pose=pose,
                radius=radius,
                length=half_length,
                material=material,
                density=density,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_sphere_collision(
        self,
        pose: sapien.Pose = sapien.Pose(),
        radius: float = 1,
        material: PhysicalMaterialRecord = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="sphere",
                pose=pose,
                radius=radius,
                material=material,
                density=density,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_convex_collision_from_file(
        self,
        filename,
        pose: sapien.Pose = sapien.Pose(),
        scale: Vec3 = [1, 1, 1],
        material: PhysicalMaterialRecord = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="convex_mesh",
                filename=filename,
                pose=pose,
                scale=scale,
                material=material,
                density=density,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_multiple_convex_collisions_from_file(
        self,
        filename,
        pose: sapien.Pose = sapien.Pose(),
        scale: Vec3 = [1, 1, 1],
        material: PhysicalMaterialRecord = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
        decomposition: typing.Literal["none", "coacd"] = "none",
        decomposition_params=dict(),
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="multiple_convex_meshes",
                filename=filename,
                pose=pose,
                scale=scale,
                material=material,
                density=density,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
                decomposition=decomposition,
                decomposition_params=decomposition_params,
            )
        )
        return self

    def add_nonconvex_collision_from_file(
        self,
        filename: str,
        pose: sapien.Pose = sapien.Pose(),
        scale: Vec3 = [1, 1, 1],
        material: PhysicalMaterialRecord = None,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        self.collision_records.append(
            CollisionShapeRecord(
                type="nonconvex_mesh",
                filename=filename,
                pose=pose,
                scale=scale,
                material=material,
                density=0,
                patch_radius=patch_radius,
                min_patch_radius=min_patch_radius,
                is_trigger=is_trigger,
            )
        )
        return self

    def add_plane_visual(
        self,
        pose: sapien.Pose = sapien.Pose(),
        scale: Vec3 = [1, 1, 1],
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is None:
            material = VisualMaterialRecord()
        if not isinstance(
            material, (VisualMaterialRecord, sapien.render.RenderMaterial)
        ):
            material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="plane", pose=pose, scale=scale, material=material, name=name
            )
        )
        return self

    def add_box_visual(
        self,
        pose: sapien.Pose = sapien.Pose(),
        half_size: Vec3 = [1, 1, 1],
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is None:
            material = VisualMaterialRecord()
        if not isinstance(
            material, (VisualMaterialRecord, sapien.render.RenderMaterial)
        ):
            material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="box", pose=pose, scale=half_size, material=material, name=name
            )
        )
        return self

    def add_capsule_visual(
        self,
        pose: sapien.Pose = sapien.Pose(),
        radius: float = 1,
        half_length: float = 1,
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is None:
            material = VisualMaterialRecord()
        if not isinstance(
            material, (VisualMaterialRecord, sapien.render.RenderMaterial)
        ):
            material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="capsule",
                pose=pose,
                radius=radius,
                length=half_length,
                material=material,
                name=name,
            )
        )
        return self

    def add_cylinder_visual(
        self,
        pose: sapien.Pose = sapien.Pose(),
        radius: float = 1,
        half_length: float = 1,
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is None:
            material = VisualMaterialRecord()
        if not isinstance(
            material, (VisualMaterialRecord, sapien.render.RenderMaterial)
        ):
            material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="cylinder",
                pose=pose,
                radius=radius,
                length=half_length,
                material=material,
                name=name,
            )
        )
        return self

    def add_sphere_visual(
        self,
        pose: sapien.Pose = sapien.Pose(),
        radius: float = 1,
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is None:
            material = VisualMaterialRecord()
        if not isinstance(
            material, (VisualMaterialRecord, sapien.render.RenderMaterial)
        ):
            material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="sphere",
                pose=pose,
                radius=radius,
                material=material,
                name=name,
            )
        )
        return self

    def add_visual_from_file(
        self,
        filename: str,
        pose: sapien.Pose = sapien.Pose(),
        scale: Vec3 = [1, 1, 1],
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is not None and not isinstance(
            material, (VisualMaterialRecord, sapien.render.RenderMaterial)
        ):
            material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="file",
                filename=filename,
                pose=pose,
                scale=scale,
                material=material,
                name=name,
            )
        )
        return self

    def add_visual_from_mesh(
        self,
        mesh: MeshRecord,
        pose: sapien.Pose = sapien.Pose(),
        scale: Vec3 = [1, 1, 1],
        material: VisualMaterialRecord = None,
        name: str = "",
    ):
        if material is not None:
            if not isinstance(
                material, (VisualMaterialRecord, sapien.render.RenderMaterial)
            ):
                material = VisualMaterialRecord(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="mesh",
                mesh=mesh,
                pose=pose,
                scale=scale,
                material=material,
                name=name,
            )
        )
        return self
