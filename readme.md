# SimulAted Physical Interactive ENvironment
A physical simulation and robotics engine based on PhysX.

## Set up
I currently do not have a detailed set up steps, but you need to do roughly the same steps.

1. Clone this repository recursively into submodules.
2. Download source code of PhysX.
3. Read my `CMakeLists` to figure out what 3rd party libraries you need.
4. Build the project following standard steps for a cmake project.
4. Avoid building the ROS dependencies if you are not sure what to do.

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

# Installation
```bash
apt install libx11-6 libassimp4 libxcb1
```

