from .viewer import Viewer
from .. import core as sapien
import numpy as np
from typing import List
import os

__global_viewer = None


def get_visual_aabb(actor: sapien.Actor):
    mins = np.ones(3) * np.inf
    maxs = -mins

    for b in actor.get_visual_bodies():
        for s in b.get_render_shapes():
            scaled_vertices = s.mesh.vertices * b.scale
            local_pose = b.local_pose
            mat = (actor.get_pose() * local_pose).to_transformation_matrix()
            world_vertices = scaled_vertices @ (mat[:3, :3].T) + mat[:3, 3]
            mins = np.minimum(mins, world_vertices.min(0))
            maxs = np.maximum(maxs, world_vertices.max(0))
    return mins, maxs


class AdhocViewer:
    def __init__(self):
        self.engine = sapien.Engine()
        self.engine.set_log_level("info")
        sapien.SapienRenderer.set_log_level("info")
        self.renderer = sapien.SapienRenderer(culling="none")
        self.engine.set_renderer(self.renderer)
        self.scene = self.engine.create_scene()
        self.viewer = Viewer(self.renderer)
        self.viewer.set_scene(self.scene)
        self.setup_lighting()

    def setup_lighting(self):
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 0, -1], [1, 1, 1])

    def clear(self):
        actors = self.scene.get_all_actors()
        for a in actors:
            self.scene.remove_actor(a)

        articulations = self.scene.get_all_articulations()
        for a in articulations:
            self.scene.remove_articulation(a)

        # TODO: add function to remove pcds
        # FIXME: clear cached resources

    def add_mesh_file(self, f: str, name=None):
        if name is None:
            name = os.path.basename(f)

        actor = (
            self.scene.create_actor_builder()
            .add_visual_from_file(f)
            .build_static(name=name)
        )
        return actor

    def add_raw_mesh(self, vertices: np.ndarray, faces: np.ndarray):
        vertices = np.ascontiguousarray(vertices.astype(np.float32)).reshape((-1, 3))
        indices = np.ascontiguousarray(faces.astype(np.uint32)).reshape((-1, 3))
        mesh = self.renderer.create_mesh(vertices, indices)
        actor = (
            self.scene.create_actor_builder().add_visual_from_mesh(mesh).build_static()
        )
        actor.get_visual_bodies()[0].shade_flat = True
        return mesh

    def add_trimesh(self, mesh):
        return self.add_mesh(mesh.vertices, mesh.faces)

    def add_raw_pcd(self, pcd: np.ndarray):
        vertices = np.ascontiguousarray(pcd.astype(np.float32)).reshape((-1, 3))
        entity = self.scene.add_particle_entity(pcd)
        return entity

    def add_trimesh_pcd(self, pcd):
        return self.add_raw_pcd(pcd.vertices)

    def get_scene_aabb(self):
        actors = self.scene.get_all_actors() + [
            l for a in self.scene.get_all_articulations() for l in a.links
        ]
        mins = np.ones(3) * np.inf
        maxs = -mins
        for a in actors:
            amin, amax = get_visual_aabb(a)
            mins = np.minimum(mins, amin)
            maxs = np.maximum(maxs, amax)

        return mins, maxs

    def focus_scene(self):
        mins, maxs = self.get_scene_aabb()

        center = (mins + maxs) / 2
        radius = np.linalg.norm(maxs - mins) / 2

        dist = radius * 2
        far = radius * 10
        near = radius * 0.1

        self.viewer.window.set_camera_parameters(near, far, np.pi / 3)
        self.viewer.set_camera_xyz(*(center - [dist, 0, 0]))
        self.viewer.set_camera_rpy(0, 0, 0)


def __get_adhoc_viewer():
    global __global_viewer
    if __global_viewer is None:
        __global_viewer = AdhocViewer()

    return __global_viewer


def is_mesh_file(anything):
    suffix = [".obj", ".glb", ".gltf", ".ply", ".dae", ".stl"]
    if not isinstance(anything, str):
        return False
    if not any(anything.lower().endswith(x) for x in suffix):
        return False
    return True


def is_raw_mesh(anything):
    if not isinstance(anything, list):
        return False
    if len(anything) != 2:
        return False

    try:
        vertices = np.ascontiguousarray(anything[0]).astype(np.float32)
        indices = np.ascontiguousarray(anything[1]).astype(np.uint32)
        if len(vertices.shape) != 2:
            return False
        if vertices.shape[1] != 3:
            return False
        if len(indices.shape) != 2:
            return False
        if indices.shape[1] != 3:
            return False
    except ValueError:
        return False

    return True


def is_raw_pcd(anything):
    try:
        vertices = np.ascontiguousarray(anything).astype(np.float32)
        if len(vertices.shape) != 2:
            return False
        if vertices.shape[1] != 3:
            return False

    except ValueError:
        return False

    return True


def is_trimesh(anything):
    try:
        import trimesh
    except ModuleNotFoundError:
        return False

    if isinsntace(anything, trimesh.Trimesh):
        return True

    return False


# TODO visualize tets?


def show_anything(*args, update_camera=True, loop=True, **kwargs):
    viewer = __get_adhoc_viewer()
    viewer.clear()

    for name, thing in list(zip([None] * len(args), args)) + list(kwargs.items()):
        if is_mesh_file(thing):
            viewer.add_mesh_file(thing, name=name)
            continue

        if is_raw_mesh(thing):
            viewer.add_raw_mesh(*anything)
            continue

        if is_trimesh(thing):
            viewer.add_trimesh(anything)
            continue

    if update_camera:
        viewer.focus_scene()

    viewer.scene.update_render()
    viewer.viewer.render()

    if loop:
        while not viewer.viewer.closed:
            viewer.scene.update_render()
            viewer.viewer.render()


if __name__ == "__main__":
    import sys

    show_anything(*sys.argv[1:])
