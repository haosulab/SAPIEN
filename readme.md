# SAPIEN: A SimulAted Part-based Interactive ENvironment
SAPIEN is a realistic and physics-rich simulated environment that hosts a
large-scale set for articulated objects. It enables various robotic vision and
interaction tasks that require detailed part-level understanding. SAPIEN is a
collaborative effort between researchers at UCSD, Stanford and SFU. The dataset
is a continuation of ShapeNet and PartNet.

## SAPIEN Engine
SAPIEN Engine provides physical simulation for articulated objects. It powers
reinforcement learning and robotics with its pure Python interface.

## SAPIEN Renderer
SAPIEN Renderer renders scenes with OpenGL rasterizer and optionally Nvidia
OptiX ray-tracer. It provides visualization/realistic rendering for the SAPIEN
environment. Currently, the ray-tracing support is only available via building
from source.

## PartNet-Mobility
SAPIEN releases PartNet-Mobility dataset, which is a collection of 2K
articulated objects with motion annotations and rendernig material. The dataset
powers research for generalizable computer vision and manipulation.

## Website and Documentation
SAPIEN Website: [https://sapien.ucsd.edu/](https://sapien.ucsd.edu/). SAPIEN
Documentation:
[https://sapien.ucsd.edu/docs/index.html](https://sapien.ucsd.edu/docs/index.html).

## Before build
`git submodule update --init --recursive`

## Build with Docker
`./docker_build_wheels.sh`

## CMake build
```bash
mkdir build
cd build
cmake -DCMake_BUILD_TYPE=Release ..
make
```
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
