#include <sapien/articulation/sapien_articulation.h>
#include <sapien/extension.h>
#include <sapien/sapien_scene.h>

#include <pybind11/pybind11.h>

class MyCallback : public sapien::SceneMultistepCallback {
  sapien::SScene *mScene;

public:
  MyCallback(sapien::SScene *scene) : mScene(scene) {}
  void beforeMultistep() override {
    printf("time step: %f\n", mScene->getTimestep());
    printf("multistep!\n");
  }
  void beforeStep(int step) override { printf("step%d\n", step); }
  void afterStep(int step) override {}
  void afterMultistep() override {}
};

namespace py = pybind11;
PYBIND11_MODULE(sapien_example_extension, m) {

  py::class_<MyCallback>(m, "MyCallback")
      .def(py::init(
          [](void *scene) { return std::make_unique<MyCallback>((sapien::SScene *)scene); }))
      .def_property_readonly("_ptr", [](MyCallback &callback) { return (void*)&callback; });
}
