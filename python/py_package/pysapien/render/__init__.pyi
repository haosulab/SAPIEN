from __future__ import annotations
import sapien.pysapien.render
import typing
import numpy
import sapien.pysapien
import sapien.pysapien.internal_renderer
_Shape = typing.Tuple[int, ...]
_T = typing.TypeVar("T")

__all__ = [
    "RenderBodyComponent",
    "RenderCameraComponent",
    "RenderCubemap",
    "RenderCudaMeshComponent",
    "RenderDirectionalLightComponent",
    "RenderImageCuda",
    "RenderLightComponent",
    "RenderMaterial",
    "RenderParallelogramLightComponent",
    "RenderPointCloudComponent",
    "RenderPointLightComponent",
    "RenderShape",
    "RenderShapeBox",
    "RenderShapeCapsule",
    "RenderShapeCylinder",
    "RenderShapePlane",
    "RenderShapeSphere",
    "RenderShapeTriangleMesh",
    "RenderShapeTriangleMeshPart",
    "RenderSpotLightComponent",
    "RenderSystem",
    "RenderTexture",
    "RenderTexture2D",
    "RenderTexturedLightComponent",
    "RenderWindow",
    "SapienRenderer",
    "clear_cache",
    "get_camera_shader_dir",
    "get_device_summary",
    "get_imgui_ini_filename",
    "get_msaa",
    "get_ray_tracing_denoiser",
    "get_ray_tracing_dof_aperture",
    "get_ray_tracing_dof_plane",
    "get_ray_tracing_path_depth",
    "get_ray_tracing_samples_per_pixel",
    "get_viewer_shader_dir",
    "load_scene",
    "set_camera_shader_dir",
    "set_imgui_ini_filename",
    "set_log_level",
    "set_msaa",
    "set_picture_format",
    "set_ray_tracing_denoiser",
    "set_ray_tracing_dof_aperture",
    "set_ray_tracing_dof_plane",
    "set_ray_tracing_path_depth",
    "set_ray_tracing_samples_per_pixel",
    "set_viewer_shader_dir"
]


class RenderBodyComponent(sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def attach(self, arg0: RenderShape) -> RenderBodyComponent: ...
    def compute_global_aabb_tight(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[2, 3]]: ...
    def disable_render_id(self) -> None: ...
    def enable_render_id(self) -> None: ...
    def get_global_aabb_fast(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[2, 3]]: ...
    @typing.overload
    def set_property(self, name: str, value: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: float) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: int) -> None: ...
    def set_texture(self, name: str, texture: RenderTexture) -> None: ...
    def set_texture_array(self, name: str, textures: list[RenderTexture]) -> None: ...
    @property
    def _internal_node(self) -> sapien.pysapien.internal_renderer.Node:
        """
        :type: sapien.pysapien.internal_renderer.Node
        """
    @property
    def is_render_id_disabled(self) -> bool:
        """
        :type: bool
        """
    @property
    def render_shapes(self) -> list[RenderShape]:
        """
        :type: list[RenderShape]
        """
    @property
    def shading_mode(self) -> int:
        """
        :type: int
        """
    @shading_mode.setter
    def shading_mode(self, arg1: int) -> None:
        pass
    @property
    def visibility(self) -> float:
        """
        :type: float
        """
    @visibility.setter
    def visibility(self, arg1: float) -> None:
        pass
    pass
class RenderCameraComponent(sapien.pysapien.Component):
    def __init__(self, width: int, height: int, shader_dir: str = '') -> None: ...
    def get_extrinsic_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[3, 4]]: 
        """
        Get 3x4 extrinsic camera matrix in OpenCV format.
        """
    def get_far(self) -> float: ...
    def get_global_pose(self) -> sapien.pysapien.Pose: ...
    def get_height(self) -> int: ...
    def get_intrinsic_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[3, 3]]: 
        """
        Get 3x3 intrinsic camera matrix in OpenCV format.
        """
    def get_local_pose(self) -> sapien.pysapien.Pose: ...
    def get_model_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: 
        """
        Get model matrix (inverse of extrinsic matrix) used in rendering (Y up, Z back)
        """
    def get_near(self) -> float: ...
    def get_picture(self, name: str) -> numpy.ndarray: ...
    def get_picture_cuda(self, name: str) -> RenderImageCuda: 
        """
        This function transfers the rendered image into a CUDA buffer.
        The returned object implements __cuda_array_interface__

        Usage:

        image = camera.getImageCuda()
        torch_tensor = torch.as_tensor(image)

        Warning: The camera must not be destroyed when the GPU tensor is in use by the
        consumer library. Make a copy if needed.
        """
    def get_picture_names(self) -> list[str]: ...
    def get_projection_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: 
        """
        Get projection matrix in used in rendering (right-handed NDC with [-1,1] XY and [0,1] Z)
        """
    def get_skew(self) -> float: ...
    def get_width(self) -> int: ...
    def set_far(self, far: float) -> None: ...
    def set_focal_lengths(self, fx: float, fy: float) -> None: ...
    def set_fovx(self, fov: float, compute_y: bool = True) -> None: ...
    def set_fovy(self, fov: float, compute_x: bool = True) -> None: ...
    def set_local_pose(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_near(self, near: float) -> None: ...
    def set_perspective_parameters(self, near: float, far: float, fx: float, fy: float, cx: float, cy: float, skew: float) -> None: ...
    def set_principal_point(self, cx: float, cy: float) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: float) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: int) -> None: ...
    def set_skew(self, skew: float) -> None: ...
    def set_texture(self, name: str, texture: RenderTexture) -> None: ...
    def set_texture_array(self, name: str, textures: list[RenderTexture]) -> None: ...
    def take_picture(self) -> None: ...
    @property
    def cx(self) -> float:
        """
        :type: float
        """
    @property
    def cy(self) -> float:
        """
        :type: float
        """
    @property
    def far(self) -> float:
        """
        :type: float
        """
    @far.setter
    def far(self, arg1: float) -> None:
        pass
    @property
    def fovx(self) -> float:
        """
        :type: float
        """
    @property
    def fovy(self) -> float:
        """
        :type: float
        """
    @property
    def fx(self) -> float:
        """
        :type: float
        """
    @property
    def fy(self) -> float:
        """
        :type: float
        """
    @property
    def global_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @property
    def height(self) -> int:
        """
        :type: int
        """
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def near(self) -> float:
        """
        :type: float
        """
    @near.setter
    def near(self, arg1: float) -> None:
        pass
    @property
    def skew(self) -> float:
        """
        :type: float
        """
    @skew.setter
    def skew(self, arg1: float) -> None:
        pass
    @property
    def width(self) -> int:
        """
        :type: int
        """
    pass
class RenderCubemap():
    @typing.overload
    def __init__(self, filename: str) -> None: ...
    @typing.overload
    def __init__(self, px: str, nx: str, py: str, ny: str, pz: str, nz: str) -> None: ...
    def export(self, filename: str) -> None: ...
    pass
class RenderCudaMeshComponent(sapien.pysapien.Component):
    def __init__(self, max_vertex_count: int, max_triangle_count: int) -> None: ...
    def get_cuda_triangles(self) -> sapien.pysapien.CudaArray: ...
    def get_cuda_vertices(self) -> sapien.pysapien.CudaArray: ...
    def get_triangle_count(self) -> int: ...
    def get_vertex_count(self) -> int: ...
    def getmaterial(self) -> RenderMaterial: ...
    def set_data_source(self, vertex_provider: sapien.pysapien.CudaDataSource) -> None: ...
    def set_material(self, material: RenderMaterial) -> None: ...
    def set_triangle_count(self, arg0: int) -> None: ...
    def set_triangles(self, arg0: numpy.ndarray[numpy.uint32, _Shape[m, 3]]) -> None: ...
    def set_vertex_count(self, arg0: int) -> None: ...
    @property
    def cuda_triangles(self) -> sapien.pysapien.CudaArray:
        """
        :type: sapien.pysapien.CudaArray
        """
    @property
    def cuda_vertices(self) -> sapien.pysapien.CudaArray:
        """
        :type: sapien.pysapien.CudaArray
        """
    @property
    def material(self) -> RenderMaterial:
        """
        :type: RenderMaterial
        """
    @material.setter
    def material(self, arg1: RenderMaterial) -> None:
        pass
    @property
    def triangle_count(self) -> int:
        """
        :type: int
        """
    @triangle_count.setter
    def triangle_count(self, arg1: int) -> None:
        pass
    @property
    def vertex_count(self) -> int:
        """
        :type: int
        """
    @vertex_count.setter
    def vertex_count(self, arg1: int) -> None:
        pass
    pass
class RenderLightComponent(sapien.pysapien.Component):
    def disable_shadow(self) -> None: ...
    def enable_shadow(self) -> None: ...
    def get_color(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_global_pose(self) -> sapien.pysapien.Pose: ...
    def get_local_pose(self) -> sapien.pysapien.Pose: ...
    def get_shadow_far(self) -> float: ...
    def get_shadow_map_size(self) -> int: ...
    def get_shadow_near(self) -> float: ...
    def set_color(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: 
        """
        color
        """
    def set_local_pose(self, pose: sapien.pysapien.Pose) -> None: ...
    def set_shadow_far(self, far: float) -> None: ...
    def set_shadow_map_size(self, size: int) -> None: ...
    def set_shadow_near(self, near: float) -> None: ...
    @property
    def color(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @color.setter
    def color(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def global_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def shadow(self) -> bool:
        """
        :type: bool
        """
    @shadow.setter
    def shadow(self, arg1: bool) -> None:
        pass
    @property
    def shadow_far(self) -> float:
        """
        :type: float
        """
    @shadow_far.setter
    def shadow_far(self, arg1: float) -> None:
        pass
    @property
    def shadow_map_size(self) -> int:
        """
        :type: int
        """
    @shadow_map_size.setter
    def shadow_map_size(self, arg1: int) -> None:
        pass
    @property
    def shadow_near(self) -> float:
        """
        :type: float
        """
    @shadow_near.setter
    def shadow_near(self, arg1: float) -> None:
        pass
    pass
class RenderImageCuda(sapien.pysapien.CudaArray):
    @property
    def __cuda_array_interface__(self) -> dict:
        """
        :type: dict
        """
    @property
    def cuda_id(self) -> int:
        """
        :type: int
        """
    @property
    def ptr(self) -> int:
        """
        :type: int
        """
    @property
    def shape(self) -> list[int]:
        """
        :type: list[int]
        """
    @property
    def strides(self) -> list[int]:
        """
        :type: list[int]
        """
    @property
    def typstr(self) -> str:
        """
        :type: str
        """
    pass
class RenderDirectionalLightComponent(RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def get_shadow_half_size(self) -> float: ...
    def set_shadow_half_size(self, arg0: float) -> None: ...
    @property
    def shadow_half_size(self) -> float:
        """
        :type: float
        """
    @shadow_half_size.setter
    def shadow_half_size(self, arg1: float) -> None:
        pass
    pass
class RenderMaterial():
    def __init__(self, emission: list[float] = [0.0, 0.0, 0.0, 0.0], base_color: list[float] = [1.0, 1.0, 1.0, 1.0], specular: float = 0.0, roughness: float = 1.0, metallic: float = 0.0, transmission: float = 0.0, ior: float = 1.4500000476837158, transmission_roughness: float = 0.0) -> None: ...
    def get_base_color(self) -> list[float]: ...
    def get_diffuse_texture(self) -> RenderTexture2D: ...
    def get_emission(self) -> list[float]: ...
    def get_emission_texture(self) -> RenderTexture2D: ...
    def get_ior(self) -> float: ...
    def get_metallic(self) -> float: ...
    def get_metallic_texture(self) -> RenderTexture2D: ...
    def get_normal_texture(self) -> RenderTexture2D: ...
    def get_roughness(self) -> float: ...
    def get_roughness_texture(self) -> RenderTexture2D: ...
    def get_specular(self) -> float: ...
    def get_transmission(self) -> float: ...
    def get_transmission_roughness(self) -> float: ...
    def get_transmission_texture(self) -> RenderTexture2D: ...
    def set_base_color(self, color: list[float]) -> None: ...
    def set_diffuse_texture(self, texture: RenderTexture2D) -> None: ...
    def set_emission(self, emission: list[float]) -> None: ...
    def set_emission_texture(self, texture: RenderTexture2D) -> None: ...
    def set_ior(self, ior: float) -> None: ...
    def set_metallic(self, metallic: float) -> None: ...
    def set_metallic_texture(self, texture: RenderTexture2D) -> None: ...
    def set_normal_texture(self, texture: RenderTexture2D) -> None: ...
    def set_roughness(self, roughness: float) -> None: ...
    def set_roughness_texture(self, texture: RenderTexture2D) -> None: ...
    def set_specular(self, specular: float) -> None: ...
    def set_transmission(self, transmission: float) -> None: ...
    def set_transmission_roughness(self, roughness: float) -> None: ...
    def set_transmission_texture(self, texture: RenderTexture2D) -> None: ...
    @property
    def base_color(self) -> list[float]:
        """
        :type: list[float]
        """
    @base_color.setter
    def base_color(self, arg1: list[float]) -> None:
        pass
    @property
    def diffuse_texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @diffuse_texture.setter
    def diffuse_texture(self, arg1: RenderTexture2D) -> None:
        pass
    @property
    def emission(self) -> list[float]:
        """
        :type: list[float]
        """
    @emission.setter
    def emission(self, arg1: list[float]) -> None:
        pass
    @property
    def emission_texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @emission_texture.setter
    def emission_texture(self, arg1: RenderTexture2D) -> None:
        pass
    @property
    def ior(self) -> float:
        """
        :type: float
        """
    @ior.setter
    def ior(self, arg1: float) -> None:
        pass
    @property
    def metallic(self) -> float:
        """
        :type: float
        """
    @metallic.setter
    def metallic(self, arg1: float) -> None:
        pass
    @property
    def metallic_texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @metallic_texture.setter
    def metallic_texture(self, arg1: RenderTexture2D) -> None:
        pass
    @property
    def normal_texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @normal_texture.setter
    def normal_texture(self, arg1: RenderTexture2D) -> None:
        pass
    @property
    def roughness(self) -> float:
        """
        :type: float
        """
    @roughness.setter
    def roughness(self, arg1: float) -> None:
        pass
    @property
    def roughness_texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @roughness_texture.setter
    def roughness_texture(self, arg1: RenderTexture2D) -> None:
        pass
    @property
    def specular(self) -> float:
        """
        :type: float
        """
    @specular.setter
    def specular(self, arg1: float) -> None:
        pass
    @property
    def transmission(self) -> float:
        """
        :type: float
        """
    @transmission.setter
    def transmission(self, arg1: float) -> None:
        pass
    @property
    def transmission_roughness(self) -> float:
        """
        :type: float
        """
    @transmission_roughness.setter
    def transmission_roughness(self, arg1: float) -> None:
        pass
    @property
    def transmission_texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @transmission_texture.setter
    def transmission_texture(self, arg1: RenderTexture2D) -> None:
        pass
    pass
class RenderParallelogramLightComponent(RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def get_angle(self) -> float: ...
    def get_half_height(self) -> float: ...
    def get_half_width(self) -> float: ...
    def set_shape(self, half_width: float, half_height: float, angle: float = 1.5707963705062866) -> None: ...
    @property
    def angle(self) -> float:
        """
        :type: float
        """
    @property
    def half_height(self) -> float:
        """
        :type: float
        """
    @property
    def half_width(self) -> float:
        """
        :type: float
        """
    pass
class RenderPointCloudComponent(sapien.pysapien.Component):
    def __init__(self, capacity: int = 0) -> None: ...
    def get_cuda_vertices(self) -> sapien.pysapien.CudaArray: ...
    def set_attribute(self, name: str, attribute: numpy.ndarray[numpy.float32, _Shape[m, n]]) -> RenderPointCloudComponent: ...
    def set_vertices(self, vertices: numpy.ndarray[numpy.float32, _Shape[m, 3]]) -> RenderPointCloudComponent: ...
    pass
class RenderPointLightComponent(RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    pass
class RenderShape():
    def get_culling(self) -> typing.Literal['back', 'front', 'none', 'both']: ...
    def get_local_pose(self) -> sapien.pysapien.Pose: ...
    def get_name(self) -> str: ...
    def get_per_scene_id(self) -> int: ...
    def set_culling(self, culling: typing.Literal['back', 'front', 'none', 'both']) -> None: ...
    def set_local_pose(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_name(self, name: str) -> None: ...
    @property
    def culling(self) -> typing.Literal['back', 'front', 'none', 'both']:
        """
        :type: typing.Literal['back', 'front', 'none', 'both']
        """
    @culling.setter
    def culling(self, arg1: typing.Literal['back', 'front', 'none', 'both']) -> None:
        pass
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @name.setter
    def name(self, arg1: str) -> None:
        pass
    @property
    def per_scene_id(self) -> int:
        """
        :type: int
        """
    pass
class RenderShapeBox(RenderShape):
    def __init__(self, half_size: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: RenderMaterial) -> None: ...
    def get_half_size(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    @property
    def half_size(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class RenderShapeCapsule(RenderShape):
    def __init__(self, radius: float, half_length: float, material: RenderMaterial) -> None: ...
    def get_half_length(self) -> float: ...
    def get_radius(self) -> float: ...
    @property
    def half_length(self) -> float:
        """
        :type: float
        """
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class RenderShapeCylinder(RenderShape):
    def __init__(self, radius: float, half_length: float, material: RenderMaterial) -> None: ...
    def get_half_length(self) -> float: ...
    def get_radius(self) -> float: ...
    @property
    def half_length(self) -> float:
        """
        :type: float
        """
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class RenderShapePlane(RenderShape):
    def __init__(self, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: RenderMaterial) -> None: ...
    def get_scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class RenderShapeSphere(RenderShape):
    def __init__(self, radius: float, material: RenderMaterial) -> None: ...
    def get_radius(self) -> float: ...
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class RenderShapeTriangleMesh(RenderShape):
    @typing.overload
    def __init__(self, vertices: numpy.ndarray[numpy.float32, _Shape[m, 3]], triangles: numpy.ndarray[numpy.uint32, _Shape[m, 3]], normals: numpy.ndarray[numpy.float32, _Shape[m, 3]], material: RenderMaterial) -> None: ...
    @typing.overload
    def __init__(self, filename: str, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]] = array([1., 1., 1.], dtype=float32), material: RenderMaterial = None) -> None: ...
    def get_filename(self) -> str: ...
    def get_parts(self) -> list[RenderShapeTriangleMeshPart]: ...
    def get_scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def set_scale(self, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: 
        """
        Note: this function only works when the shape is not added to scene
        """
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def parts(self) -> list[RenderShapeTriangleMeshPart]:
        """
        :type: list[RenderShapeTriangleMeshPart]
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @scale.setter
    def scale(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    pass
class RenderShapeTriangleMeshPart():
    def get_cuda_triangles(self) -> sapien.pysapien.CudaArray: ...
    def get_cuda_vertices(self) -> sapien.pysapien.CudaArray: ...
    def get_material(self) -> RenderMaterial: ...
    def get_triangles(self) -> numpy.ndarray[numpy.uint32, _Shape[m, 3]]: ...
    def get_vertices(self) -> numpy.ndarray[numpy.float32, _Shape[m, 3]]: ...
    @property
    def cuda_triangles(self) -> sapien.pysapien.CudaArray:
        """
        :type: sapien.pysapien.CudaArray
        """
    @property
    def cuda_vertices(self) -> sapien.pysapien.CudaArray:
        """
        :type: sapien.pysapien.CudaArray
        """
    @property
    def material(self) -> RenderMaterial:
        """
        :type: RenderMaterial
        """
    @property
    def triangles(self) -> numpy.ndarray[numpy.uint32, _Shape[m, 3]]:
        """
        :type: numpy.ndarray[numpy.uint32, _Shape[m, 3]]
        """
    @property
    def vertices(self) -> numpy.ndarray[numpy.float32, _Shape[m, 3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape[m, 3]]
        """
    pass
class RenderSpotLightComponent(RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def get_inner_fov(self) -> float: ...
    def get_outer_fov(self) -> float: ...
    def set_inner_fov(self, fov: float) -> None: ...
    def set_outer_fov(self, fov: float) -> None: ...
    @property
    def inner_fov(self) -> float:
        """
        :type: float
        """
    @inner_fov.setter
    def inner_fov(self, arg1: float) -> None:
        pass
    @property
    def outer_fov(self) -> float:
        """
        :type: float
        """
    @outer_fov.setter
    def outer_fov(self, arg1: float) -> None:
        pass
    pass
class RenderSystem(sapien.pysapien.System):
    def __init__(self) -> None: ...
    def get_ambient_light(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_cameras(self) -> list[RenderCameraComponent]: ...
    def get_cubemap(self) -> RenderCubemap: ...
    def get_lights(self) -> list[RenderLightComponent]: ...
    def get_render_bodies(self) -> list[RenderBodyComponent]: ...
    def set_ambient_light(self, color: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    def set_cubemap(self, cubemap: RenderCubemap) -> None: ...
    @property
    def _internal_scene(self) -> sapien.pysapien.internal_renderer.Scene:
        """
        :type: sapien.pysapien.internal_renderer.Scene
        """
    @property
    def ambient_light(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @ambient_light.setter
    def ambient_light(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def cameras(self) -> list[RenderCameraComponent]:
        """
        :type: list[RenderCameraComponent]
        """
    @property
    def cubemap(self) -> RenderCubemap:
        """
        :type: RenderCubemap
        """
    @cubemap.setter
    def cubemap(self, arg1: RenderCubemap) -> None:
        pass
    @property
    def lights(self) -> list[RenderLightComponent]:
        """
        :type: list[RenderLightComponent]
        """
    @property
    def render_bodies(self) -> list[RenderBodyComponent]:
        """
        :type: list[RenderBodyComponent]
        """
    pass
class RenderTexture():
    def __init__(self, array: numpy.ndarray, dim: int, format: str, mipmap_levels: int = 1, filter_mode: typing.Literal['nearest', 'linear'] = 'linear', address_mode: typing.Literal['repeat', 'border', 'edge', 'mirror'] = 'repeat', srgb: bool = False) -> None: ...
    def download(self) -> numpy.ndarray: ...
    def get_address_mode(self) -> typing.Literal['repeat', 'border', 'edge', 'mirror']: ...
    def get_channels(self) -> int: ...
    def get_depth(self) -> int: ...
    def get_filter_mode(self) -> typing.Literal['nearest', 'linear']: ...
    def get_format(self) -> str: ...
    def get_height(self) -> int: ...
    def get_mipmap_levels(self) -> int: ...
    def get_width(self) -> int: ...
    def upload(self, data: numpy.ndarray) -> None: ...
    @property
    def address_mode(self) -> typing.Literal['repeat', 'border', 'edge', 'mirror']:
        """
        :type: typing.Literal['repeat', 'border', 'edge', 'mirror']
        """
    @property
    def channels(self) -> int:
        """
        :type: int
        """
    @property
    def depth(self) -> int:
        """
        :type: int
        """
    @property
    def filter_mode(self) -> typing.Literal['nearest', 'linear']:
        """
        :type: typing.Literal['nearest', 'linear']
        """
    @property
    def format(self) -> str:
        """
        :type: str
        """
    @property
    def height(self) -> int:
        """
        :type: int
        """
    @property
    def is_srgb(self) -> bool:
        """
        :type: bool
        """
    @property
    def mipmap_levels(self) -> int:
        """
        :type: int
        """
    @property
    def width(self) -> int:
        """
        :type: int
        """
    pass
class RenderTexture2D():
    def __init__(self, filename: str, mipmap_levels: int = 1, filter_mode: typing.Literal['nearest', 'linear'] = 'linear', address_mode: typing.Literal['repeat', 'border', 'edge', 'mirror'] = 'repeat') -> None: ...
    def download(self) -> numpy.ndarray: ...
    def get_address_mode(self) -> typing.Literal['repeat', 'border', 'edge', 'mirror']: ...
    def get_channels(self) -> int: ...
    def get_filename(self) -> str: ...
    def get_filter_mode(self) -> typing.Literal['nearest', 'linear']: ...
    def get_format(self) -> str: ...
    def get_height(self) -> int: ...
    def get_mipmap_levels(self) -> int: ...
    def get_width(self) -> int: ...
    def upload(self, data: numpy.ndarray) -> None: ...
    @property
    def address_mode(self) -> typing.Literal['repeat', 'border', 'edge', 'mirror']:
        """
        :type: typing.Literal['repeat', 'border', 'edge', 'mirror']
        """
    @property
    def channels(self) -> int:
        """
        :type: int
        """
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def filter_mode(self) -> typing.Literal['nearest', 'linear']:
        """
        :type: typing.Literal['nearest', 'linear']
        """
    @property
    def format(self) -> str:
        """
        :type: str
        """
    @property
    def height(self) -> int:
        """
        :type: int
        """
    @property
    def mipmap_levels(self) -> int:
        """
        :type: int
        """
    @property
    def width(self) -> int:
        """
        :type: int
        """
    pass
class RenderTexturedLightComponent(RenderSpotLightComponent, RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def get_texture(self) -> RenderTexture2D: ...
    def set_texture(self, texture: RenderTexture2D) -> None: ...
    @property
    def texture(self) -> RenderTexture2D:
        """
        :type: RenderTexture2D
        """
    @texture.setter
    def texture(self, arg1: RenderTexture2D) -> None:
        pass
    pass
class RenderWindow():
    def __init__(self, width: int, height: int, shader_dir: str) -> None: ...
    def get_camera_pose(self) -> sapien.pysapien.Pose: ...
    def get_camera_position(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]: ...
    def get_camera_projection_matrix(self) -> numpy.ndarray[numpy.float32]: ...
    def get_camera_property_float(self, arg0: str) -> float: ...
    def get_camera_property_int(self, arg0: str) -> int: ...
    def get_camera_rotation(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[4]]: ...
    def get_content_scale(self) -> float: ...
    def get_picture(self, name: str) -> numpy.ndarray: ...
    def get_picture_pixel(self, name: str, x: int, y: int) -> numpy.ndarray: ...
    def get_picture_size(self, name: str) -> list[int]: ...
    def hide(self) -> None: ...
    def key_down(self, key: str) -> bool: ...
    def key_press(self, key: str) -> bool: ...
    def mouse_click(self, key: int) -> bool: ...
    def mouse_down(self, key: int) -> bool: ...
    def render(self, target_name: str, ui_windows: list[sapien.pysapien.internal_renderer.UIWidget] = []) -> None: ...
    def resize(self, width: int, height: int) -> None: ...
    def set_camera_parameters(self, near: float, far: float, fovy: float) -> None: ...
    def set_camera_pose(self, arg0: sapien.pysapien.Pose) -> None: ...
    def set_camera_position(self, position: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    @typing.overload
    def set_camera_property(self, key: str, value: float) -> None: ...
    @typing.overload
    def set_camera_property(self, key: str, value: int) -> None: ...
    def set_camera_rotation(self, quat: numpy.ndarray[numpy.float32, _Shape, _Shape[4]]) -> None: ...
    def set_camera_texture(self, name: str, texture: RenderTexture2D) -> None: ...
    def set_camera_texture_array(self, name: str, textures: list[RenderTexture2D]) -> None: ...
    def set_drop_callback(self, callback: typing.Callable[[list[str]], None]) -> None: ...
    def set_focus_callback(self, callback: typing.Callable[[int], None]) -> None: ...
    def set_intrinsic_parameters(self, near: float, far: float, fx: float, fy: float, cx: float, cy: float, skew: float) -> None: ...
    def set_scene(self, scene: sapien.pysapien.Scene) -> None: ...
    def set_shader_dir(self, shader_dir: str) -> None: ...
    def show(self) -> None: ...
    def unset_drop_callback(self) -> None: ...
    def unset_focus_callback(self) -> None: ...
    @property
    def _internal_renderer(self) -> sapien.pysapien.internal_renderer.Renderer:
        """
        :type: sapien.pysapien.internal_renderer.Renderer
        """
    @property
    def alt(self) -> bool:
        """
        :type: bool
        """
    @property
    def ctrl(self) -> bool:
        """
        :type: bool
        """
    @property
    def cursor(self) -> bool:
        """
        :type: bool
        """
    @cursor.setter
    def cursor(self, arg1: bool) -> None:
        pass
    @property
    def denoiser(self) -> typing.Literal['none', 'oidn', 'optix']:
        """
        :type: typing.Literal['none', 'oidn', 'optix']
        """
    @denoiser.setter
    def denoiser(self, arg1: typing.Literal['none', 'oidn', 'optix']) -> None:
        pass
    @property
    def display_picture_names(self) -> list[str]:
        """
        Names for available display targets that can be displayed in the render function

        :type: list[str]
        """
    @property
    def far(self) -> float:
        """
        :type: float
        """
    @property
    def fovy(self) -> float:
        """
        :type: float
        """
    @property
    def fps(self) -> float:
        """
        :type: float
        """
    @property
    def mouse_delta(self) -> list[float]:
        """
        :type: list[float]
        """
    @property
    def mouse_position(self) -> list[float]:
        """
        :type: list[float]
        """
    @property
    def mouse_wheel_delta(self) -> list[float]:
        """
        :type: list[float]
        """
    @property
    def near(self) -> float:
        """
        :type: float
        """
    @property
    def shift(self) -> bool:
        """
        :type: bool
        """
    @property
    def should_close(self) -> bool:
        """
        :type: bool
        """
    @property
    def size(self) -> list[int]:
        """
        :type: list[int]
        """
    @property
    def super(self) -> bool:
        """
        :type: bool
        """
    pass
class SapienRenderer():
    def __init__(self, offscreen_only: bool = False, max_num_materials: int = 128, max_num_textures: int = 512, default_mipmap_levels: int = 1, default_device: str = '', do_not_load_texture: bool = False) -> None: ...
    @property
    def _internal_context(self) -> sapien.pysapien.internal_renderer.Context:
        """
        :type: sapien.pysapien.internal_renderer.Context
        """
    pass
def _internal_set_shader_search_path(arg0: str) -> None:
    pass
def clear_cache(models: bool = True, images: bool = True, shaders: bool = False) -> None:
    pass
def get_camera_shader_dir() -> str:
    pass
def get_device_summary() -> str:
    pass
def get_imgui_ini_filename() -> str:
    pass
def get_msaa() -> int:
    pass
def get_ray_tracing_denoiser() -> typing.Literal['none', 'oidn', 'optix']:
    pass
def get_ray_tracing_dof_aperture() -> float:
    pass
def get_ray_tracing_dof_plane() -> float:
    pass
def get_ray_tracing_path_depth() -> int:
    pass
def get_ray_tracing_samples_per_pixel() -> int:
    pass
def get_viewer_shader_dir() -> str:
    pass
def load_scene(filename: str, apply_scale: bool = True) -> dict:
    pass
def set_camera_shader_dir(dir: str) -> None:
    pass
def set_imgui_ini_filename(filename: str) -> None:
    pass
def set_log_level(level: str) -> None:
    pass
def set_msaa(msaa: int) -> None:
    pass
def set_picture_format(name: str, format: str) -> None:
    pass
def set_ray_tracing_denoiser(name: typing.Literal['none', 'oidn', 'optix']) -> None:
    pass
def set_ray_tracing_dof_aperture(radius: float) -> None:
    pass
def set_ray_tracing_dof_plane(depth: float) -> None:
    pass
def set_ray_tracing_path_depth(depth: int) -> None:
    pass
def set_ray_tracing_samples_per_pixel(spp: int) -> None:
    pass
def set_viewer_shader_dir(dir: str) -> None:
    pass
