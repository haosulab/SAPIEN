# SAPIEN
SAPIEN is a realistic and physics-rich simulated environment that hosts a
large-scale set for articulated objects. It enables various robotic vision and
interaction tasks that require detailed part-level understanding. SAPIEN is a
collaborative effort between researchers at UCSD, Stanford and SFU. The dataset
is a continuation of ShapeNet and PartNet.

## Getting Started
SAPIEN is distributed via [PyPI](https://pypi.org/project/sapien/). Installation is just

```shell
pip install sapien
```

It requires Linux with NVIDIA, AMD, or Intel GPU to run. Verify installation with

```shell
python -m sapien.exapmle.hello_world
```

Next, follow our tutorial at:
[https://sapien.ucsd.edu/docs/latest/index.html](https://sapien.ucsd.edu/docs/latest/index.html).

### Offscreen rendering on a server
To use SAPIEN on a GPU server without display, the only system dependencies
required are `libegl1` and `libxext6`. If using NVIDIA docker environment,
enable graphics, utility, and compute by setting the environment variable in the
Dockerfile.
```Dockerfile
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
```

### Virtual desktop on a server
To use SAPIEN on a GPU server with virtual display, additionally install `xvfb`,
`x11vnc`, and any window manager such as `fluxbox` or `xfce`. Add display
capabilities for NVIDIA docker.
```Dockerfile
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute,display
```

Assuming `fluxbox`, start a VNC server by 
```shell
x11vnc -create -env FD_PROG=/usr/bin/fluxbox  -env X11VNC_FINDDISPLAY_ALWAYS_FAILS=1 -env X11VNC_CREATE_GEOM=${99:-1920x1080x16} -gone 'pkill Xvfb' -nopw
# Note: you should use a strong password and/or only allow local access
```
Now you can connect to the server at port 5900. SAPIEN should be fully functional, test with
```shell
python -m sapien.exapmle.hello_world
```

## Change Log
<details open> <summary>3.0</summary>

- Major API & infrastructure overhaul

<details> 
<summary>2.2</summary>

- Rename `VulkanRenderer` to `SapienRenderer` (VulkanRenderer is still an alias)
- Support **ray tracing** in `SapienRenderer`
- Deprecate `KuafuRenderer`, use the rt shader in `SapienRenderer` instead
- **GPU-accelerated stereo depth sensor simulation**
- **Render server**
- Python 3.11
- bug fixes
  - Fix inverse kinematics default active joint mask (now defaults to all 1s)
  - Fix incorrectly exported memory in Vulkan-Cuda interop
  - Fix joint `get_global_pose`
</details>

<details>
<summary>2.1</summary>

- Python 3.10
- Bug fixes
  - crash when not using renderer
  - joint force limit (was impulse limit)
  - incorrect inertia computation in scaled URDF
  - incorrect point-light shadow
  - incorrect collision when loaded from dae
- Utility improvements
  - set_material
  - active light
  - flat shading
  - dynamic point rendering
  - envmap generation
  - multi-thread envs

</details>

<details>
<summary>2.1</summary>

- Refactor light system
  - Remove light functions on scene.renderer_scene
- Refactor camera system
  - Cameras no longer require mounts
  - Camera can change its mount and mounted pose by `camera.set_parent` and
    `camera.set_local_pose`.
  - When camera is not mounted, setting local pose is setting its global pose.
  - Add functions `scene.add_camera` and `scene.remove_camera`
  - `add_mounted_camera` can be replaced with `add_camera` followed by
    `camera.set_parent` and `camera.set_local_pose`. `add_mounted_camera` is
    still provided but fovx should not longer be provided.
  - Remove functions related to mount, including `find_camera_by_mount`.
  - Cameras now support full camera parameters through `camera.near`,
    `camera.far`, `camera.set_fovx`, `camera.set_fovy`,
    `camera.set_focal_lengths`, `camera.set_principal_point`, `camera.skew`, and
    the all-in-one method `camera.set_perspective_parameters`.
- Refactor render shape system
  - Originally, after `actor.get_visual_bodies()` and
    `visual_body.get_render_shapes()`, users typically do `shape.scale` and
    `shape.pose`. These are no longer valid. It is required to check
    `visual_body.type`. When `type` is `mesh`, `shape.scale` is replaced with
    `visual_body.scale` and `shape.pose` is replaced by
    `visual_body.local_pose`. These changes are made to match `add_visual_shape`
    functions when building the actor.
</details>

<details>
<summary>pre2.0</summary>

- Shader change: 4th component in default camera shader now gives the 0-1 depth value.
- Add "critical" and "off" log levels.
- Add support for pointcloud and line rendering (for visualizing camera and point cloud)
- Performance: the same shader only compile once per process
- Bug fix
  - Articulation setDriveTarget was now correctly reversed for prismatic joint (joint setDriveTarget is not affected)
  - Fix kinematic articulation loader
</details>

<details>
<summary>1 to 2 migration</summary>

- replace `scene.renderer_scene.add_xxx_light` with `scene.add_xxx_light`
- replace `scene.remove_mounted_camera` with `scene.remove_camera`
- optionally, remove `fovx` from `scene.add_mounted_camera`.
</details>


<details>
<summary>1.1</summary>

- Support nonconvex static/kinematic collision shape
- Add warning for small mass/inertia
- Introduce Entity as the base class of Actors
- Add Light classes inherited from entity, allowing manipulate light objects in sapien scene
- Updates to the viewer
  - rename actor to entity when appropriate
- Partial support the material tag in URDF loader (primitive shape, single color)
- Bug fixes for the renderer
- Support inner and outer FOV for spotlight
</details>

<details>
<summary>1.0</summary>

- Replace the old Vulkan based renderer completely
  - See `sapien.core.renderer` for details
- Expose GUI functionalities to Python
- Reimplement Vulkan viewer in Python 
- Expose PhysX shape wrapper to Python. For example,
  - Collision shapes can be retrieved through `actor.get_collision_shapes`
  - Collision groups on a shape can be set by `CollisionShape.set_collision_groups`
  - Shapes are now also available in `Contact`.
- API changes
  - Render material creation is now `renderer.create_material()`
  - in actor builder: `add_xxx_shape` is replaced with `add_xxx_collision`.
  - move light functions from scene to `scene.renderer_scene`
- Add centrifugal and Coriolis force.
- Change default physical parameters for better stability.
</details>

## Website and Documentation
SAPIEN Website: [https://sapien.ucsd.edu/](https://sapien.ucsd.edu/). SAPIEN
Documentation:
[https://sapien.ucsd.edu/docs/latest/index.html](https://sapien.ucsd.edu/docs/latest/index.html).

## Build from source
### Before build
Make sure all submodules are initialized `git submodule update --init --recursive`.

### Build with Docker
To build SAPIEN, simply run `./docker_build_wheels.sh`. It is not recommended to
build outside of our provided docker.

For reference, the Dockerfile is provided [here](/docker/Dockerfile). Note that
PhysX needs to be compiled with clang-9 into static libraries before building
the Docker image.

### Build without Docker
It can be tricky to setup all dependencies outside of a Docker environment. You
need to install all dependencies according to the [Docker
environment](/docker/Dockerfile). If all dependencies set up correctly, run
`python setup.py bdist_wheel` to build the wheel.

## Cite SAPIEN
If you use SAPIEN and its assets, please cite the following works:
```
@InProceedings{Xiang_2020_SAPIEN,
author = {Xiang, Fanbo and Qin, Yuzhe and Mo, Kaichun and Xia, Yikuan and Zhu, Hao and Liu, Fangchen and Liu, Minghua and Jiang, Hanxiao and Yuan, Yifu and Wang, He and Yi, Li and Chang, Angel X. and Guibas, Leonidas J. and Su, Hao},
title = {{SAPIEN}: A SimulAted Part-based Interactive ENvironment},
booktitle = {The IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
month = {June},
year = {2020}}
```
```
@InProceedings{Mo_2019_CVPR,
author = {Mo, Kaichun and Zhu, Shilin and Chang, Angel X. and Yi, Li and Tripathi, Subarna and Guibas, Leonidas J. and Su, Hao},
title = {{PartNet}: A Large-Scale Benchmark for Fine-Grained and Hierarchical Part-Level {3D} Object Understanding},
booktitle = {The IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
month = {June},
year = {2019}
}
```
```
@article{chang2015shapenet,
title={Shapenet: An information-rich 3d model repository},
author={Chang, Angel X and Funkhouser, Thomas and Guibas, Leonidas and Hanrahan, Pat and Huang, Qixing and Li, Zimo and Savarese, Silvio and Savva, Manolis and Song, Shuran and Su, Hao and others},
journal={arXiv preprint arXiv:1512.03012},
year={2015}
}
```
If you use SAPIEN Realistic Depth generated by SAPIEN's simulated depth sensor, please cite the following work:
```
@ARTICLE{10027470,
  author={Zhang, Xiaoshuai and Chen, Rui and Li, Ang and Xiang, Fanbo and Qin, Yuzhe and Gu, Jiayuan and Ling, Zhan and Liu, Minghua and Zeng, Peiyu and Han, Songfang and Huang, Zhiao and Mu, Tongzhou and Xu, Jing and Su, Hao},
  journal={IEEE Transactions on Robotics}, 
  title={Close the Optical Sensing Domain Gap by Physics-Grounded Active Stereo Sensor Simulation}, 
  year={2023},
  volume={},
  number={},
  pages={1-19},
  doi={10.1109/TRO.2023.3235591}}
```
