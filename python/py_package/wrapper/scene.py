from __future__ import annotations

from typing import Optional, TypeVar, Union
from warnings import warn

from .. import pysapien as sapien
from ..pysapien import Scene as _Scene
from ..pysapien.physx import PhysxSceneConfig as SceneConfig
from ..pysapien.render import RenderCameraComponent, RenderCubemap


class Widget:
    def __init__(self):
        warn(
            "Inheriting from sapien.Widget is not required",
            DeprecationWarning,
            stacklevel=2,
        )


class Scene(_Scene):
    def __init__(self, systems=None):
        if systems is None:
            super().__init__(
                [sapien.physx.PhysxCpuSystem(), sapien.render.RenderSystem()]
            )
        else:
            super().__init__(systems)

    @property
    def timestep(self):
        return self.physx_system.timestep

    @timestep.setter
    def timestep(self, timestep):
        self.physx_system.timestep = timestep

    def set_timestep(self, timestep):
        self.timestep = timestep

    def get_timestep(self):
        return self.timestep

    def create_actor_builder(self):
        from .actor_builder import ActorBuilder

        return ActorBuilder().set_scene(self)

    def create_articulation_builder(self):
        from .articulation_builder import ArticulationBuilder

        return ArticulationBuilder().set_scene(self)

    def create_urdf_loader(self):
        from .urdf_loader import URDFLoader

        loader = URDFLoader()
        loader.set_scene(self)
        return loader

    def create_physical_material(
        self, static_friction: float, dynamic_friction: float, restitution: float
    ):
        return sapien.physx.PhysxMaterial(
            static_friction, dynamic_friction, restitution
        )

    def remove_actor(self, actor):
        self.remove_entity(actor)
        pass

    def remove_articulation(self, articulation):
        entities = [l.entity for l in articulation.links]
        for e in entities:
            self.remove_entity(e)

    # TODO: find actor by id
    def add_camera(
        self, name, width: int, height: int, fovy: float, near: float, far: float
    ) -> RenderCameraComponent:
        camera_mount = sapien.Entity()
        camera = RenderCameraComponent(width, height)
        camera.set_fovy(fovy, compute_x=True)
        camera.near = near
        camera.far = far
        camera_mount.add_component(camera)
        self.add_entity(camera_mount)
        camera_mount.name = name
        camera.name = name

        return camera

    def add_mounted_camera(
        self, name, mount, pose, width, height, fovy, near, far
    ) -> RenderCameraComponent:
        camera = RenderCameraComponent(width, height)
        camera.set_fovy(fovy, compute_x=True)
        camera.near = near
        camera.far = far
        mount.add_component(camera)
        camera.local_pose = pose
        camera.name = name

        return camera

    def remove_camera(self, camera):
        self.remove_entity(camera.entity)

    def get_cameras(self):
        return self.render_system.cameras

    def get_mounted_cameras(self):
        return self.get_cameras()

    def step(self):
        self.physx_system.step()

    def update_render(self):
        self.render_system.step()

    def add_ground(
        self,
        altitude,
        render=True,
        material=None,
        render_material=None,
        render_half_size=[10, 10],
    ):
        from .actor_builder import ActorBuilder

        builder = self.create_actor_builder()
        if render:
            builder.add_plane_visual(
                sapien.Pose(p=[0, 0, altitude], q=[0.7071068, 0, -0.7071068, 0]),
                [10, *render_half_size],
                render_material,
                "",
            )

        builder.add_plane_collision(
            sapien.Pose(p=[0, 0, altitude], q=[0.7071068, 0, -0.7071068, 0]),
            material,
        )
        builder.set_physx_body_type("static")
        ground = builder.build()
        ground.name = "ground"
        return ground

    def get_contacts(self):
        return self.physx_system.get_contacts()

    def get_all_actors(self):
        return [
            c.entity
            for c in self.physx_system.rigid_dynamic_components
            + self.physx_system.rigid_static_components
        ]

    def get_all_articulations(self):
        return [
            c.articulation
            for c in self.physx_system.articulation_link_components
            if c.is_root
        ]

    def create_drive(
        self,
        body0: Optional[Union[sapien.Entity, sapien.physx.PhysxRigidBaseComponent]],
        pose0: sapien.Pose,
        body1: Union[sapien.Entity, sapien.physx.PhysxRigidBaseComponent],
        pose1: sapien.Pose,
    ):
        if body0 is None:
            c0 = None
        elif isinstance(body0, sapien.Entity):
            c0 = next(
                c
                for c in body0.components
                if isinstance(c, sapien.physx.PhysxRigidBaseComponent)
            )
        else:
            c0 = body0

        assert body1 is not None
        if isinstance(body1, sapien.Entity):
            e1 = body1
            c1 = next(
                c
                for c in body1.components
                if isinstance(c, sapien.physx.PhysxRigidBaseComponent)
            )
        else:
            e1 = body1.entity
            c1 = body1

        drive = sapien.physx.PhysxDriveComponent(c1)
        drive.parent = c0
        drive.pose_in_child = pose1
        drive.pose_in_parent = pose0
        e1.add_component(drive)
        return drive

    def create_connection(
        self,
        body0: Optional[Union[sapien.Entity, sapien.physx.PhysxRigidBaseComponent]],
        pose0: sapien.Pose,
        body1: Union[sapien.Entity, sapien.physx.PhysxRigidBaseComponent],
        pose1: sapien.Pose,
    ):
        if body0 is None:
            c0 = None
        elif isinstance(body0, sapien.Entity):
            c0 = next(
                c
                for c in body0.components
                if isinstance(c, sapien.physx.PhysxRigidBaseComponent)
            )
        else:
            c0 = body0

        assert body1 is not None
        if isinstance(body1, sapien.Entity):
            e1 = body1
            c1 = next(
                c
                for c in body1.components
                if isinstance(c, sapien.physx.PhysxRigidBaseComponent)
            )
        else:
            e1 = body1.entity
            c1 = body1

        connection = sapien.physx.PhysxDistanceJointComponent(c1)
        connection.parent = c0
        connection.pose_in_child = pose1
        connection.pose_in_parent = pose0
        e1.add_component(connection)
        connection.set_limit(0, 0)
        return connection

    def create_gear(
        self,
        body0: Optional[Union[sapien.Entity, sapien.physx.PhysxRigidBaseComponent]],
        pose0: sapien.Pose,
        body1: Union[sapien.Entity, sapien.physx.PhysxRigidBaseComponent],
        pose1: sapien.Pose,
    ):
        if body0 is None:
            c0 = None
        elif isinstance(body0, sapien.Entity):
            c0 = next(
                c
                for c in body0.components
                if isinstance(c, sapien.physx.PhysxRigidBaseComponent)
            )
        else:
            c0 = body0

        assert body1 is not None
        if isinstance(body1, sapien.Entity):
            e1 = body1
            c1 = next(
                c
                for c in body1.components
                if isinstance(c, sapien.physx.PhysxRigidBaseComponent)
            )
        else:
            e1 = body1.entity
            c1 = body1

        gear = sapien.physx.PhysxGearComponent(c1)
        gear.parent = c0
        gear.pose_in_child = pose1
        gear.pose_in_parent = pose0
        e1.add_component(gear)
        return gear

    @property
    def render_id_to_visual_name(self):
        # TODO
        return

    @property
    def ambient_light(self):
        return self.render_system.ambient_light

    @ambient_light.setter
    def ambient_light(self, color):
        self.render_system.ambient_light = color

    def set_ambient_light(self, color):
        self.ambient_light = color

    def add_point_light(
        self,
        position,
        color,
        shadow=False,
        shadow_near=0.1,
        shadow_far=10.0,
        shadow_map_size=2048,
    ):
        entity = sapien.Entity()
        light = sapien.render.RenderPointLightComponent()
        entity.add_component(light)
        light.color = color
        light.shadow = shadow
        light.shadow_near = shadow_near
        light.shadow_far = shadow_far
        light.shadow_map_size = shadow_map_size
        light.pose = sapien.Pose(position)
        self.add_entity(entity)
        return light

    def add_directional_light(
        self,
        direction,
        color,
        shadow=False,
        position=[0, 0, 0],
        shadow_scale=10.0,
        shadow_near=-10.0,
        shadow_far=10.0,
        shadow_map_size=2048,
    ):
        entity = sapien.Entity()
        light = sapien.render.RenderDirectionalLightComponent()
        entity.add_component(light)
        light.color = color
        light.shadow = shadow
        light.shadow_near = shadow_near
        light.shadow_far = shadow_far
        light.shadow_half_size = shadow_scale
        light.shadow_map_size = shadow_map_size
        light.pose = sapien.Pose(
            position, sapien.math.shortest_rotation([1, 0, 0], direction)
        )
        self.add_entity(entity)
        return light

    def add_spot_light(
        self,
        position,
        direction,
        inner_fov: float,
        outer_fov: float,
        color,
        shadow=False,
        shadow_near=0.1,
        shadow_far=10.0,
        shadow_map_size=2048,
    ):
        entity = sapien.Entity()
        light = sapien.render.RenderSpotLightComponent()
        entity.add_component(light)
        light.color = color
        light.shadow = shadow
        light.shadow_near = shadow_near
        light.shadow_far = shadow_far
        light.shadow_map_size = shadow_map_size
        light.inner_fov = inner_fov
        light.outer_fov = outer_fov
        light.pose = sapien.Pose(
            position, sapien.math.shortest_rotation([1, 0, 0], direction)
        )
        self.add_entity(entity)
        return light

    def add_area_light_for_ray_tracing(
        self, pose: sapien.Pose, color, half_width: float, half_height: float
    ):
        entity = sapien.Entity()
        light = sapien.render.RenderParallelogramLightComponent()
        entity.add_component(light)
        light.set_shape(half_width, half_height)
        light.color = color
        light.pose = pose
        self.add_entity(entity)
        return light

    # TODO: textured light

    def remove_light(self, light):
        self.remove_entity(light.entity)

    def set_environment_map(self, cubemap: str | RenderCubemap):
        if isinstance(cubemap, str):
            self.render_system.cubemap = sapien.render.RenderCubemap(cubemap)
        else:
            self.render_system.cubemap = cubemap

    def set_environment_map_from_files(
        self, px: str, nx: str, py: str, ny: str, pz: str, nz: str
    ):
        self.render_system.cubemap = sapien.render.RenderCubemap(px, nx, py, ny, pz, nz)

    # TODO particle entity, deformable entity

    def create_viewer(self):
        from sapien.utils import Viewer

        viewer = Viewer()
        viewer.set_scene(self)

        return viewer

    def __del__(self):
        self.clear()
