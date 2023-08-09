from __future__ import annotations
import sapien.pysapien.render
import typing
import numpy
import sapien.pysapien
import sapien.pysapien.internal_renderer
_Shape = typing.Tuple[int, ...]

__all__ = [
    "RenderBodyComponent",
    "RenderCameraComponent",
    "RenderCubemap",
    "RenderDirectionalLightComponent",
    "RenderLightComponent",
    "RenderMaterial",
    "RenderParallelogramLightComponent",
    "RenderPointLightComponent",
    "RenderShape",
    "RenderShapeBox",
    "RenderShapeCapsule",
    "RenderShapePlane",
    "RenderShapeSphere",
    "RenderShapeTriangleMesh",
    "RenderShapeTriangleMeshPart",
    "RenderSpotLightComponent",
    "RenderSystem",
    "RenderTargetCuda",
    "RenderTexture",
    "RenderTexture2D",
    "RenderTexturedLightComponent",
    "RenderWindow",
    "SapienRenderer",
    "clear_cache",
    "get_camera_shader_dir",
    "get_msaa",
    "get_ray_tracing_path_depth",
    "get_ray_tracing_samples_per_pixel",
    "get_ray_tracing_use_denoiser",
    "get_viewer_shader_dir",
    "set_camera_shader_dir",
    "set_log_level",
    "set_msaa",
    "set_picture_format",
    "set_ray_tracing_path_depth",
    "set_ray_tracing_samples_per_pixel",
    "set_ray_tracing_use_denoiser",
    "set_viewer_shader_dir"
]


class RenderBodyComponent(sapien.pysapien.Component):
    def __init__(self) -> None: ...
    def attach(self, arg0: RenderShape) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: float) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: int) -> None: ...
    def set_texture(self, name: str, texture: RenderTexture) -> None: ...
    def set_texture_array(self, name: str, textures: typing.List[RenderTexture]) -> None: ...
    @property
    def _internal_node(self) -> sapien.pysapien.internal_renderer.Node:
        """
        :type: sapien.pysapien.internal_renderer.Node
        """
    @property
    def render_shapes(self) -> typing.List[RenderShape]:
        """
        :type: typing.List[RenderShape]
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
    def get_intrinsic_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[3, 3]]: 
        """
        Get 3x3 intrinsic camera matrix in OpenCV format.
        """
    def get_model_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: 
        """
        Get model matrix (inverse of extrinsic matrix) used in rendering (Y up, Z back)
        """
    def get_picture(self, name: str) -> numpy.ndarray: ...
    def get_picture_cuda(self, name: str) -> RenderTargetCuda: 
        """
        This function transfers the rendered image into a CUDA buffer.
        The returned object implements __cuda_array_interface__

        Usage:

        image = camera.getImageCuda()
        torch_tensor = torch.as_tensor(image)

        Warning: The camera must not be destroyed when the GPU tensor is in use by the
        consumer library. Make a copy if needed.
        """
    def get_picture_names(self) -> typing.List[str]: ...
    def get_projection_matrix(self) -> numpy.ndarray[numpy.float32, _Shape[4, 4]]: 
        """
        Get projection matrix in used in rendering (right-handed NDC with [-1,1] XY and [0,1] Z)
        """
    def set_focal_lengths(self, fx: float, fy: float) -> None: ...
    def set_fovx(self, fov: float, compute_y: bool = True) -> None: ...
    def set_fovy(self, fov: float, compute_x: bool = True) -> None: ...
    def set_perspective_parameters(self, near: float, far: float, fx: float, fy: float, cx: float, cy: float, skew: float) -> None: ...
    def set_principal_point(self, cx: float, cy: float) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: float) -> None: ...
    @typing.overload
    def set_property(self, name: str, value: int) -> None: ...
    def set_texture(self, name: str, texture: RenderTexture) -> None: ...
    def set_texture_array(self, name: str, textures: typing.List[RenderTexture]) -> None: ...
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
class RenderLightComponent(sapien.pysapien.Component):
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
class RenderDirectionalLightComponent(RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
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
    def __init__(self, emission: typing.List[float] = [0.0, 0.0, 0.0, 0.0], base_color: typing.List[float] = [1.0, 1.0, 1.0, 1.0], specular: float = 0.0, roughness: float = 1.0, metallic: float = 0.0, transmission: float = 0.0, ior: float = 1.4500000476837158, transmission_roughness: float = 0.0) -> None: ...
    def get_base_color(self) -> typing.List[float]: ...
    def get_diffuse_texture(self) -> RenderTexture2D: ...
    def get_emission(self) -> typing.List[float]: ...
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
    def set_base_color(self, arg0: typing.List[float]) -> None: ...
    def set_diffuse_texture(self, arg0: RenderTexture2D) -> None: ...
    def set_emission(self, arg0: typing.List[float]) -> None: ...
    def set_emission_texture(self, arg0: RenderTexture2D) -> None: ...
    def set_ior(self, arg0: float) -> None: ...
    def set_metallic(self, arg0: float) -> None: ...
    def set_metallic_texture(self, arg0: RenderTexture2D) -> None: ...
    def set_normal_texture(self, arg0: RenderTexture2D) -> None: ...
    def set_roughness(self, arg0: float) -> None: ...
    def set_roughness_texture(self, arg0: RenderTexture2D) -> None: ...
    def set_specular(self, arg0: float) -> None: ...
    def set_transmission(self, arg0: float) -> None: ...
    def set_transmission_roughness(self, arg0: float) -> None: ...
    def set_transmission_texture(self, arg0: RenderTexture2D) -> None: ...
    @property
    def base_color(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @base_color.setter
    def base_color(self, arg1: typing.List[float]) -> None:
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
    def emission(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @emission.setter
    def emission(self, arg1: typing.List[float]) -> None:
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
    @property
    def edge0(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @edge0.setter
    def edge0(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    @property
    def edge1(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    @edge1.setter
    def edge1(self, arg1: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None:
        pass
    pass
class RenderPointLightComponent(RenderLightComponent, sapien.pysapien.Component):
    def __init__(self) -> None: ...
    pass
class RenderShape():
    @property
    def local_pose(self) -> sapien.pysapien.Pose:
        """
        :type: sapien.pysapien.Pose
        """
    @local_pose.setter
    def local_pose(self, arg1: sapien.pysapien.Pose) -> None:
        pass
    pass
class RenderShapeBox(RenderShape):
    def __init__(self, half_size: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: RenderMaterial) -> None: ...
    @property
    def half_size(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class RenderShapeCapsule(RenderShape):
    def __init__(self, radius: float, half_length: float, material: RenderMaterial) -> None: ...
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
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class RenderShapeSphere(RenderShape):
    def __init__(self, radius: float, material: RenderMaterial) -> None: ...
    @property
    def radius(self) -> float:
        """
        :type: float
        """
    pass
class RenderShapeTriangleMesh(RenderShape):
    @typing.overload
    def __init__(self, filename: str, scale: numpy.ndarray[numpy.float32, _Shape, _Shape[3]], material: RenderMaterial = None) -> None: ...
    @typing.overload
    def __init__(self, vertices: numpy.ndarray[numpy.float32, _Shape[m, 3]], triangles: numpy.ndarray[numpy.uint32, _Shape[m, 3]], normals: numpy.ndarray[numpy.float32, _Shape[m, 3]], material: RenderMaterial) -> None: ...
    @property
    def filename(self) -> str:
        """
        :type: str
        """
    @property
    def parts(self) -> typing.List[RenderShapeTriangleMeshPart]:
        """
        :type: typing.List[RenderShapeTriangleMeshPart]
        """
    @property
    def scale(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[3]]:
        """
        :type: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]
        """
    pass
class RenderShapeTriangleMeshPart():
    @property
    def __cuda_array_interface__(self) -> dict:
        """
        :type: dict
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
    def set_ambient_light(self, arg0: numpy.ndarray[numpy.float32, _Shape, _Shape[3]]) -> None: ...
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
    def cameras(self) -> typing.List[RenderCameraComponent]:
        """
        :type: typing.List[RenderCameraComponent]
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
    def lights(self) -> typing.List[RenderLightComponent]:
        """
        :type: typing.List[RenderLightComponent]
        """
    @property
    def render_bodies(self) -> typing.List[RenderBodyComponent]:
        """
        :type: typing.List[RenderBodyComponent]
        """
    pass
class RenderTargetCuda():
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
    def ptr(self) -> int:
        """
        :type: int
        """
    @property
    def width(self) -> int:
        """
        :type: int
        """
    pass
class RenderTexture():
    def __init__(self, array: numpy.ndarray, dim: int, format: str, mipmap_levels: int = 1, filter_mode: typing.Literal['nearest', 'linear'] = 'linear', address_mode: typing.Literal['repeat', 'border', 'edge', 'mirror'] = 'repeat', srgb: bool = False) -> None: ...
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
    def mipmap_levels(self) -> int:
        """
        :type: int
        """
    @property
    def srgb(self) -> bool:
        """
        :type: bool
        """
    @property
    def width(self) -> int:
        """
        :type: int
        """
    pass
class RenderTexture2D():
    def __init__(self, filename: str, mipmap_levels: int = 1, filter_mode: typing.Literal['nearest', 'linear'] = 'linear', address_mode: typing.Literal['repeat', 'border', 'edge', 'mirror'] = 'repeat') -> None: ...
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
    def get_camera_rotation(self) -> numpy.ndarray[numpy.float32, _Shape, _Shape[4]]: ...
    def get_content_scale(self) -> float: ...
    def get_float_texture(self, name: str) -> numpy.ndarray[numpy.float32]: ...
    def get_float_texture_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.float32]: ...
    def get_target_size(self, name: str) -> typing.List[int]: ...
    def get_uint32_texture(self, name: str) -> numpy.ndarray[numpy.uint32]: ...
    def get_uint32_texture_pixel(self, name: str, x: int, y: int) -> numpy.ndarray[numpy.uint32]: ...
    def hide(self) -> None: ...
    def key_down(self, key: str) -> bool: ...
    def key_press(self, key: str) -> bool: ...
    def mouse_click(self, key: int) -> bool: ...
    def mouse_down(self, key: int) -> bool: ...
    def render(self, target_name: str, ui_windows: typing.List[sapien.pysapien.internal_renderer.UIWidget] = []) -> None: ...
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
    def set_camera_texture_array(self, name: str, textures: typing.List[RenderTexture2D]) -> None: ...
    def set_drop_callback(self, callback: typing.Callable[[typing.List[str]], None]) -> None: ...
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
    def display_target_names(self) -> typing.List[str]:
        """
        Names for available display targets that can be displayed in the render function

        :type: typing.List[str]
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
    def mouse_delta(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @property
    def mouse_position(self) -> typing.List[float]:
        """
        :type: typing.List[float]
        """
    @property
    def mouse_wheel_delta(self) -> typing.List[float]:
        """
        :type: typing.List[float]
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
    def size(self) -> typing.List[int]:
        """
        :type: typing.List[int]
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
def get_msaa() -> int:
    pass
def get_ray_tracing_path_depth() -> int:
    pass
def get_ray_tracing_samples_per_pixel() -> int:
    pass
def get_ray_tracing_use_denoiser() -> bool:
    pass
def get_viewer_shader_dir() -> str:
    pass
def set_camera_shader_dir(dir: str) -> None:
    pass
def set_log_level(level: str) -> None:
    pass
def set_msaa(msaa: int) -> None:
    pass
def set_picture_format(name: str, format: str) -> None:
    pass
def set_ray_tracing_path_depth(depth: int) -> None:
    pass
def set_ray_tracing_samples_per_pixel(spp: int) -> None:
    pass
def set_ray_tracing_use_denoiser(enable: bool) -> None:
    pass
def set_viewer_shader_dir(dir: str) -> None:
    pass
