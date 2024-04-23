from .. import pysapien as sapien
from dataclasses import dataclass
import numpy as np
import typing
from typing import List, Union, Dict, Any, Tuple, Literal

from .coacd import do_coacd


def preprocess_mesh_file(filename: str):
    """
    Process input mesh file to a SAPIEN supported format
    Args:
        filename: input mesh file
    Returns:
        filename for the generated file or original filename
    """

    from .geometry.usd import convert_usd_to_glb

    if any(filename.lower().endswith(s) for s in [".usd", ".usda", ".usdc", ".usdz"]):
        glb_filename = filename + ".sapien.glb"
        convert_usd_to_glb(filename, glb_filename)
        return glb_filename

    return filename


@dataclass
class CollisionShapeRecord:
    type: Literal[
        "convex_mesh",
        "multiple_convex_meshes",
        "nonconvex_mesh",
        "plane",
        "box",
        "capsule",
        "sphere",
        "cylinder",
    ]

    # for mesh type
    filename: str = ""

    # mesh & box
    scale: Tuple = (1, 1, 1)

    # circle & capsule
    radius: float = 1
    # capsule
    length: float = 1

    material: Union[sapien.physx.PhysxMaterial, None] = None
    pose: sapien.Pose = sapien.Pose()

    density: float = 1000
    patch_radius: float = 0
    min_patch_radius: float = 0
    is_trigger: bool = False

    decomposition: str = "none"
    decomposition_params: Union[Dict[str, Any], None] = None


@dataclass
class VisualShapeRecord:
    type: Literal["file", "plane", "box", "capsule", "sphere", "cylinder"]

    filename: str = ""
    scale: tuple = (1, 1, 1)

    radius: float = 1
    length: float = 1

    material: Union[
        sapien.render.RenderMaterial, None
    ] = None  # None is only used for mesh

    pose: sapien.Pose = sapien.Pose()
    name: str = ""


Vec3 = Tuple


class ActorBuilder:
    def __init__(self):
        self.collision_records: List[CollisionShapeRecord] = []
        self.visual_records: List[VisualShapeRecord] = []
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

    def set_initial_pose(self, pose):
        self.initial_pose = pose
        return self

    def set_mass_and_inertia(self, mass, cmass_local_pose, inertia):
        self._mass = mass
        self._cmass_local_pose = cmass_local_pose
        self._inertia = inertia
        self._auto_inertial = False
        return self

    def set_name(self, name):
        self.name = name
        return self

    def set_scene(self, scene: sapien.Scene):
        self.scene = scene
        return self

    def set_physx_body_type(self, type):
        assert type in ["dynamic", "static", "kinematic"]
        self.physx_body_type = type
        return self

    def build_render_component(self):
        component = sapien.render.RenderBodyComponent()
        for r in self.visual_records:
            if r.type != "file":
                assert isinstance(r.material, sapien.render.RenderMaterial)
            else:
                assert r.material is None or isinstance(
                    r.material, sapien.render.RenderMaterial
                )

            if r.type == "plane":
                shape = sapien.render.RenderShapePlane(r.scale, r.material)
            elif r.type == "box":
                shape = sapien.render.RenderShapeBox(r.scale, r.material)
            elif r.type == "sphere":
                shape = sapien.render.RenderShapeSphere(r.radius, r.material)
            elif r.type == "capsule":
                shape = sapien.render.RenderShapeCapsule(r.radius, r.length, r.material)
            elif r.type == "cylinder":
                shape = sapien.render.RenderShapeCylinder(
                    r.radius, r.length, r.material
                )
            elif r.type == "file":
                shape = sapien.render.RenderShapeTriangleMesh(
                    preprocess_mesh_file(r.filename), r.scale, r.material
                )
                if r.scale[0] * r.scale[1] * r.scale[2] < 0:
                    shape.set_front_face("clockwise")
            else:
                raise Exception(f"invalid visual shape type [{r.type}]")

            shape.local_pose = r.pose
            shape.name = r.name
            component.attach(shape)
        return component

    def build_physx_component(self, link_parent=None):
        for r in self.collision_records:
            assert isinstance(r.material, sapien.physx.PhysxMaterial)

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
            try:
                if r.type == "plane":
                    shape = sapien.physx.PhysxCollisionShapePlane(
                        material=r.material,
                    )
                    shapes = [shape]
                elif r.type == "box":
                    shape = sapien.physx.PhysxCollisionShapeBox(
                        half_size=r.scale, material=r.material
                    )
                    shapes = [shape]
                elif r.type == "capsule":
                    shape = sapien.physx.PhysxCollisionShapeCapsule(
                        radius=r.radius,
                        half_length=r.length,
                        material=r.material,
                    )
                    shapes = [shape]
                elif r.type == "cylinder":
                    shape = sapien.physx.PhysxCollisionShapeCylinder(
                        radius=r.radius,
                        half_length=r.length,
                        material=r.material,
                    )
                    shapes = [shape]
                elif r.type == "sphere":
                    shape = sapien.physx.PhysxCollisionShapeSphere(
                        radius=r.radius,
                        material=r.material,
                    )
                    shapes = [shape]
                elif r.type == "convex_mesh":
                    shape = sapien.physx.PhysxCollisionShapeConvexMesh(
                        filename=preprocess_mesh_file(r.filename),
                        scale=r.scale,
                        material=r.material,
                    )
                    shapes = [shape]
                elif r.type == "nonconvex_mesh":
                    shape = sapien.physx.PhysxCollisionShapeTriangleMesh(
                        filename=preprocess_mesh_file(r.filename),
                        scale=r.scale,
                        material=r.material,
                    )
                    shapes = [shape]
                elif r.type == "multiple_convex_meshes":
                    if r.decomposition == "coacd":
                        params = r.decomposition_params
                        if params is None:
                            params = dict()

                        filename = do_coacd(preprocess_mesh_file(r.filename), **params)
                    else:
                        filename = preprocess_mesh_file(r.filename)

                    shapes = sapien.physx.PhysxCollisionShapeConvexMesh.load_multiple(
                        filename=filename,
                        scale=r.scale,
                        material=r.material,
                    )
                else:
                    raise RuntimeError(f"invalid collision shape type [{r.type}]")
            except RuntimeError as e:
                # ignore runtime error (e.g., failed to cooke mesh)
                continue

            for shape in shapes:
                shape.local_pose = r.pose
                shape.set_collision_groups(self.collision_groups)
                shape.set_density(r.density)
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
        entity.name = self.name
        entity.pose = self.initial_pose  # set pose before adding to scene
        self.scene.add_entity(entity)
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
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        half_size: Vec3 = (1, 1, 1),
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        scale: Vec3 = (1, 1, 1),
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        scale: Vec3 = (1, 1, 1),
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        density: float = 1000,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
        decomposition: typing.Literal["none", "coacd"] = "none",
        decomposition_params=dict(),
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        scale: Vec3 = (1, 1, 1),
        material: Union[sapien.physx.PhysxMaterial, None] = None,
        patch_radius: float = 0,
        min_patch_radius: float = 0,
        is_trigger: bool = False,
    ):
        if material is None:
            material = sapien.physx.get_default_material()

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
        scale: Vec3 = (1, 1, 1),
        material: Union[sapien.render.RenderMaterial, None, Vec3] = None,
        name: str = "",
    ):
        if material is None:
            material = sapien.render.RenderMaterial()
        if not isinstance(material, sapien.render.RenderMaterial):
            material = sapien.render.RenderMaterial(base_color=(*material[:3], 1))

        self.visual_records.append(
            VisualShapeRecord(
                type="plane", pose=pose, scale=scale, material=material, name=name
            )
        )
        return self

    def add_box_visual(
        self,
        pose: sapien.Pose = sapien.Pose(),
        half_size: Vec3 = (1, 1, 1),
        material: Union[sapien.render.RenderMaterial, None, Vec3] = None,
        name: str = "",
    ):
        if material is None:
            material = sapien.render.RenderMaterial()
        if not isinstance(material, sapien.render.RenderMaterial):
            material = sapien.render.RenderMaterial(base_color=(*material[:3], 1))

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
        material: Union[sapien.render.RenderMaterial, None, Vec3] = None,
        name: str = "",
    ):
        if material is None:
            material = sapien.render.RenderMaterial()
        if not isinstance(material, sapien.render.RenderMaterial):
            material = sapien.render.RenderMaterial(base_color=(*material[:3], 1))

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
        material: Union[sapien.render.RenderMaterial, None, Vec3] = None,
        name: str = "",
    ):
        if material is None:
            material = sapien.render.RenderMaterial()
        if not isinstance(material, sapien.render.RenderMaterial):
            material = sapien.render.RenderMaterial(base_color=(*material[:3], 1))

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
        material: Union[sapien.render.RenderMaterial, None, Vec3] = None,
        name: str = "",
    ):
        if material is None:
            material = sapien.render.RenderMaterial()
        if not isinstance(material, sapien.render.RenderMaterial):
            material = sapien.render.RenderMaterial(base_color=(*material[:3], 1))

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
        scale: Vec3 = (1, 1, 1),
        material: Union[sapien.render.RenderMaterial, None, Vec3] = None,
        name: str = "",
    ):
        if material is not None and not isinstance(
            material, sapien.render.RenderMaterial
        ):
            material = sapien.render.RenderMaterial(base_color=(*material[:3], 1))

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
