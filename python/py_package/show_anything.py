# from .viewer import Viewer
# from .. import core as sapien
# import numpy as np
# from typing import List
import os
import sapien
import numpy as np


class AnythingViewer:
    def __init__(self):
        self.scene = sapien.Scene()
        self.viewer = self.scene.create_viewer()
        self._setup_lighting()

    def _setup_lighting(self):
        self.scene.set_ambient_light([0.3, 0.3, 0.3])
        self.scene.add_directional_light([1, 1, -1], [1, 1, 1])

    def add_mesh_file(self, filename):
        entity = (
            sapien.Entity()
            .add_component(
                sapien.render.RenderBodyComponent().attach(
                    sapien.render.RenderShapeTriangleMesh(filename)
                )
            )
            .add_to_scene(self.scene)
        )

    def add_urdf_file(self, filename):
        loader = self.scene.create_urdf_loader()
        loader.load(filename)

    def get_scene_aabb(self):
        aabb = np.array([[1e10] * 3, [-1e10] * 3])
        for b in self.scene.render_system.render_bodies:
            aabb_ = b.get_global_aabb_fast()
            aabb = np.minimum(aabb[0], aabb_[0]), np.maximum(aabb[1], aabb_[1])
        return aabb

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


# class AdhocViewer:
#     def __init__(self):
#         self.engine = sapien.Engine()
#         self.engine.set_log_level("info")
#         sapien.SapienRenderer.set_log_level("info")
#         self.renderer = sapien.SapienRenderer(culling="none")
#         self.engine.set_renderer(self.renderer)
#         self.scene = self.engine.create_scene()
#         self.viewer = Viewer(self.renderer)
#         self.viewer.set_scene(self.scene)
#         self.setup_lighting()

#     def setup_lighting(self):
#         self.scene.set_ambient_light([0.5, 0.5, 0.5])
#         self.scene.add_directional_light([0, 0, -1], [1, 1, 1])

#     def clear(self):
#         actors = self.scene.get_all_actors()
#         for a in actors:
#             self.scene.remove_actor(a)

#         articulations = self.scene.get_all_articulations()
#         for a in articulations:
#             self.scene.remove_articulation(a)

#         # TODO: add function to remove pcds
#         # FIXME: clear cached resources

#     def add_mesh_file(self, f: str, name=None):
#         if name is None:
#             name = os.path.basename(f)

#         actor = (
#             self.scene.create_actor_builder()
#             .add_visual_from_file(f)
#             .build_static(name=name)
#         )
#         return actor

#     def add_raw_mesh(self, vertices: np.ndarray, faces: np.ndarray):
#         vertices = np.ascontiguousarray(vertices.astype(np.float32)).reshape((-1, 3))
#         indices = np.ascontiguousarray(faces.astype(np.uint32)).reshape((-1, 3))
#         mesh = self.renderer.create_mesh(vertices, indices)
#         actor = (
#             self.scene.create_actor_builder().add_visual_from_mesh(mesh).build_static()
#         )
#         actor.get_visual_bodies()[0].shade_flat = True
#         return mesh

#     def add_trimesh(self, mesh):
#         return self.add_mesh(mesh.vertices, mesh.faces)

#     def add_raw_pcd(self, pcd: np.ndarray):
#         vertices = np.ascontiguousarray(pcd.astype(np.float32)).reshape((-1, 3))
#         entity = self.scene.add_particle_entity(pcd)
#         return entity

#     def add_trimesh_pcd(self, pcd):
#         return self.add_raw_pcd(pcd.vertices)

#     def get_scene_aabb(self):
#         actors = self.scene.get_all_actors() + [
#             l for a in self.scene.get_all_articulations() for l in a.links
#         ]
#         mins = np.ones(3) * np.inf
#         maxs = -mins
#         for a in actors:
#             amin, amax = get_visual_aabb(a)
#             mins = np.minimum(mins, amin)
#             maxs = np.maximum(maxs, amax)

#         return mins, maxs

#     def focus_scene(self):
#         mins, maxs = self.get_scene_aabb()

#         center = (mins + maxs) / 2
#         radius = np.linalg.norm(maxs - mins) / 2

#         dist = radius * 2
#         far = radius * 10
#         near = radius * 0.1

#         self.viewer.window.set_camera_parameters(near, far, np.pi / 3)
#         self.viewer.set_camera_xyz(*(center - [dist, 0, 0]))
#         self.viewer.set_camera_rpy(0, 0, 0)


# def __get_adhoc_viewer():
#     global __global_viewer
#     if __global_viewer is None:
#         __global_viewer = AdhocViewer()

#     return __global_viewer


def is_mesh_file(anything):
    suffix = [".obj", ".glb", ".gltf", ".ply", ".dae", ".stl"]
    if not isinstance(anything, str):
        return False
    if not any(anything.lower().endswith(x) for x in suffix):
        return False
    return True


def is_urdf_file(anything):
    if not isinstance(anything, str):
        return False
    if not anything.lower().endswith(".urdf"):
        return False
    return True


def show_anything(*args, update_camera=True, loop=True):
    viewer = AnythingViewer()

    for thing in args:
        if is_mesh_file(thing):
            viewer.add_mesh_file(thing)
            continue

        if is_urdf_file(thing):
            viewer.add_urdf_file(thing)
            continue

    if update_camera:
        viewer.focus_scene()

    viewer.scene.update_render()
    viewer.viewer.render()

    if loop:
        while not viewer.viewer.closed:
            viewer.scene.step()
            viewer.scene.update_render()
            viewer.viewer.render()


if __name__ == "__main__":
    import sys

    if len(sys.argv) == 1:
        exit(1)

    show_anything(*sys.argv[1:])
