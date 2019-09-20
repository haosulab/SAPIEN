# PhysX Robotics simulation environment
4127 Robotics simulation environment based on PhysX. Anyone reading this, please
propose a memorable name for it!

## Set up
I currently do not have a detailed set up steps, but you need to do roughly the same steps.

1. Download source code of PhysX.
2. Read my `CMakeLists` to figure out what 3rd party libraries you need and modify the path accordingly.
3. For the `optifuser` library (created by fbxiang), you can find it at https://github.com/fbxiang/optifuser. Follow the compilation instructions, and install without OptiX support.

## Coding style guide
This is only a general guideline
### Pointer usage
Pointer ownership. An object **owns** a pointer if the pointed object should be
de-allocated when the object is destroyed. If a pointer is shared between
multiple objects, it is only de-allocated when all its owners are destroyed.

Use `std::unique_ptr` when the pointer is supposed to be owned by only 1 object.
Usually, this pointer is returns when its creator does not care about how it is
used afterwards. This pointer usually has equivalent or even better performance
to the native pointer (when optimized by compiler).

Use `std::shared_ptr` only when the pointer will be owned by multiple objects,
or when `std::unique_ptr` cannot cover your needs. Shared pointer can be slow
and hard to optimize by the compiler.

Use native pointer `Class *` as a form of weak pointer when you are sure the
content of this pointer will not be destroyed while you are using it. If it can,
use `std::weak_ptr` so you can test its validity or prolong its life.

`shared_from_this` is pretty easy to mess up so please consider avoiding it.

In general, you do not need explicit `new` and `delete` so memory leak should
not occur. **Avoid ownership loops**. They do leak.

# TODO List
## Finish Articulation
Articulation is required for precise robot control. Please read detailed
[documentation](https://gameworksdocs.nvidia.com/PhysX/4.0/documentation/PhysXGuide/Manual/Articulations.html).
To read this documentation, you need to first understand basic PhysX and PhysX
Joints. This documentation still omits a lot of details, please ask developers
directly.

## Finish URDF Loading
URDF is a simple universal file format for robot loading. We will convert all robots into URDF. We need to first finish supporting basic URDF and then consider several common extensions, e.g. Gazebo.

## Renderer
We need a fast, off-the-shelf, easy to use renderer.
