# SAPIEN: A SimulAted Part-based Interactive ENvironment
SAPIEN is a realistic and physics-rich simulated environment that hosts a
large-scale set for articulated objects. It enables various robotic vision and
interaction tasks that require detailed part-level understanding. SAPIEN is a
collaborative effort between researchers at UCSD, Stanford and SFU. The dataset
is a continuation of ShapeNet and PartNet.

## Change Log
### 1.0
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

## SAPIEN Engine
SAPIEN Engine provides physical simulation for articulated objects. It powers
reinforcement learning and robotics with its pure Python interface.

## SAPIEN Renderer
SAPIEN provides rasterized and ray traced (available soon) rendering with
Vulkan. For machines without Vulkan support, we provide a OpenGL renderer as a
second option.

## PartNet-Mobility
SAPIEN releases PartNet-Mobility dataset, which is a collection of 2K
articulated objects with motion annotations and rendernig material. The dataset
powers research for generalizable computer vision and manipulation.

## Website and Documentation
SAPIEN Website: [https://sapien.ucsd.edu/](https://sapien.ucsd.edu/). SAPIEN
Documentation:
[https://sapien.ucsd.edu/docs/index.html](https://sapien.ucsd.edu/docs/index.html).

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
If you use SAPIEN and its assets, please cite the following works.
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
