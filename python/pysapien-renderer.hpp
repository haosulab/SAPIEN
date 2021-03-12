#pragma once
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <svulkan2/core/context.h>
#include <svulkan2/scene/scene.h>
#include <svulkan2/ui/ui.h>

namespace py = pybind11;

using namespace svulkan2;

void buildRenderer(py::module &parent) {
  py::module m = parent.def_submodule("renderer");

  auto PyContext = py::class_<core::Context>(m, "Context");
  auto PyScene = py::class_<scene::Scene>(m, "Scene");
  auto PySceneNode = py::class_<scene::Node>(m, "Node");
  auto PySceneObject = py::class_<scene::Object, scene::Node>(m, "Object");
  auto PyTexture =
      py::class_<resource::SVTexture, std::shared_ptr<resource::SVTexture>>(m, "Texture");
  auto PyMaterial =
      py::class_<resource::SVMetallicMaterial, std::shared_ptr<resource::SVMetallicMaterial>>(
          m, "Material");
  auto PyModel = py::class_<resource::SVModel, std::shared_ptr<resource::SVModel>>(m, "Model");
  auto PyShape = py::class_<resource::SVShape, std::shared_ptr<resource::SVShape>>(m, "Shape");
  auto PyMesh = py::class_<resource::SVMesh, std::shared_ptr<resource::SVMesh>>(m, "Mesh");

  auto PyUIWidget = py::class_<ui::Widget, std::shared_ptr<ui::Widget>>(m, "UIWidget");
  auto PyUIWindow = py::class_<ui::Window, ui::Widget, std::shared_ptr<ui::Window>>(m, "UIWindow");
  auto PyUICheckbox =
      py::class_<ui::Checkbox, ui::Widget, std::shared_ptr<ui::Checkbox>>(m, "UICheckbox");
  auto PyUIButton = py::class_<ui::Button, ui::Widget, std::shared_ptr<ui::Button>>(m, "UIButton");
  auto PyUIRadioButtonGroup =
      py::class_<ui::RadioButtonGroup, ui::Widget, std::shared_ptr<ui::RadioButtonGroup>>(
          m, "UIRadioButtonGroup");

  auto PyUIDisplayText = py::class_<ui::DisplayText, ui::Widget, std::shared_ptr<ui::DisplayText>>(
      m, "UIDisplayText");
  auto PyUIInputText =
      py::class_<ui::InputText, ui::Widget, std::shared_ptr<ui::InputText>>(m, "UIInputText");
  auto PyUIInputFloat =
      py::class_<ui::InputFloat, ui::Widget, std::shared_ptr<ui::InputFloat>>(m, "UIInputFloat");
  auto PyUIInputFloat2 = py::class_<ui::InputFloat2, ui::Widget, std::shared_ptr<ui::InputFloat2>>(
      m, "UIInputFloat2");
  auto PyUIInputFloat3 = py::class_<ui::InputFloat3, ui::Widget, std::shared_ptr<ui::InputFloat3>>(
      m, "UIInputFloat3");
  auto PyUIInputFloat4 = py::class_<ui::InputFloat4, ui::Widget, std::shared_ptr<ui::InputFloat4>>(
      m, "UIInputFloat4");

  auto PyUISliderFloat = py::class_<ui::SliderFloat, ui::Widget, std::shared_ptr<ui::SliderFloat>>(
      m, "UISliderFloat");
  auto PyUISliderAngle = py::class_<ui::SliderAngle, ui::Widget, std::shared_ptr<ui::SliderAngle>>(
      m, "UISliderAngle");

  auto PyUISection =
      py::class_<ui::Section, ui::Widget, std::shared_ptr<ui::Section>>(m, "UISection");

  PyUIWidget.def("remove", &ui::Widget::remove);
  // UI
  PyUIWindow.def("Label", &ui::Window::Label, py::arg("label"))
      .def(py::init<>())
      .def(
          "Pos",
          [](ui::Window &window, float x, float y) {
            return window.Pos({x, y});
          },
          py::arg("x"), py::arg("y"))
      .def(
          "Size",
          [](ui::Window &window, float x, float y) {
            return window.Size({x, y});
          },
          py::arg("x"), py::arg("y"))
      .def("append", [](ui::Window &window, py::args args) {
        if (args.size() == 0) {
          throw std::runtime_error("append must take 1 or more arguments");
        }
        std::shared_ptr<ui::Window> result;
        for (auto &arg : args) {
          auto widget = arg.cast<std::shared_ptr<ui::Widget>>();
          result = window.append(widget);
        }
        return result;
      });
  PyUISection.def(py::init<>())
      .def("Label", &ui::Section::Label, py::arg("label"))
      .def("Expanded", &ui::Section::Expanded, py::arg("expanded"))
      .def("append", [](ui::Section &section, py::args args) {
        if (args.size() == 0) {
          throw std::runtime_error("append must take 1 or more arguments");
        }
        std::shared_ptr<ui::Section> result;
        for (auto &arg : args) {
          auto widget = arg.cast<std::shared_ptr<ui::Widget>>();
          result = section.append(widget);
        }
        return result;
      });

  PyUIButton.def(py::init<>())
      .def("Label", &ui::Button::Label, py::arg("label"))
      .def("Callback", &ui::Button::Callback, py::arg("func"));

  PyUICheckbox.def(py::init<>())
      .def("Label", &ui::Checkbox::Label, py::arg("label"))
      .def("Checked", &ui::Checkbox::Checked, py::arg("checked"))
      .def("Callback", &ui::Checkbox::Callback, py::arg("func"))
      .def_property_readonly("checked", &ui::Checkbox::get);
  PyUIRadioButtonGroup.def(py::init<>())
      .def("Index", &ui::RadioButtonGroup::Index, py::arg("index"))
      .def("Labels", &ui::RadioButtonGroup::Labels, py::arg("labels"))
      .def("Callback", &ui::RadioButtonGroup::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::RadioButtonGroup::get)
      .def_property_readonly("index", &ui::RadioButtonGroup::getIndex);

  PyUIDisplayText.def(py::init<>()).def("Text", &ui::DisplayText::Text, py::arg("text"));

  PyUIInputText.def(py::init<>())
      .def("Label", &ui::InputText::Label, py::arg("label"))
      .def("Size", &ui::InputText::Size, py::arg("size"));

  PyUIInputFloat.def(py::init<>())
      .def("Label", &ui::InputFloat::Label, py::arg("label"))
      .def(
          "Value", [](ui::InputFloat &input, float x) { return input.Value(x); }, py::arg("value"))
      .def("get", [](ui::InputFloat &input) { return input.get(); });

  PyUIInputFloat2.def(py::init<>())
      .def("Label", &ui::InputFloat2::Label, py::arg("label"))
      .def(
          "Value",
          [](ui::InputFloat2 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1)});
          },
          py::arg("value"))
      .def("get", [](ui::InputFloat2 &input) {
        glm::vec2 v = input.get();
        return py::array_t<float>(2u, &v[0]);
      });

  PyUIInputFloat3.def(py::init<>())
      .def("Label", &ui::InputFloat3::Label, py::arg("label"))
      .def(
          "Value",
          [](ui::InputFloat3 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1), x.at(2)});
          },
          py::arg("value"))
      .def("get", [](ui::InputFloat3 &input) {
        glm::vec3 v = input.get();
        return py::array_t<float>(3u, &v[0]);
      });

  PyUIInputFloat4.def(py::init<>())
      .def("Label", &ui::InputFloat4::Label, py::arg("label"))
      .def(
          "Value",
          [](ui::InputFloat4 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1), x.at(2), x.at(3)});
          },
          py::arg("value"))
      .def("get", [](ui::InputFloat4 &input) {
        glm::vec4 v = input.get();
        return py::array_t<float>(4u, &v[0]);
      });

  PyUISliderFloat.def(py::init<>())
      .def("Label", &ui::SliderFloat::Label, py::arg("label"))
      .def("Min", &ui::SliderFloat::Min, py::arg("min"))
      .def("Max", &ui::SliderFloat::Max, py::arg("max"))
      .def("Value", &ui::SliderFloat::Value, py::arg("value"))
      .def("Callback", &ui::SliderFloat::Callback, py::arg("func"))
      .def("get", &ui::SliderFloat::get);

  PyUISliderAngle.def(py::init<>())
      .def("Label", &ui::SliderAngle::Label, py::arg("label"))
      .def("Min", &ui::SliderAngle::Min, py::arg("min"))
      .def("Max", &ui::SliderAngle::Max, py::arg("max"))
      .def("Value", &ui::SliderAngle::Value, py::arg("value"))
      .def("Callback", &ui::SliderAngle::Callback, py::arg("func"))
      .def("get", &ui::SliderAngle::get);
  // end UI

  PyContext
      .def(py::init([](uint32_t maxNumMaterials, uint32_t maxNumTextures,
                       uint32_t defaultMipmapLevels) {
             return new core::Context(VK_API_VERSION_1_1, true, maxNumMaterials, maxNumTextures,
                                      defaultMipmapLevels);
           }),
           py::return_value_policy::automatic, py::arg("max_num_materials") = 5000,
           py::arg("max_num_textures") = 5000, py::arg("default_mipmap_levels") = 1)
      .def(
          "create_material",
          [](core::Context &context, py::array_t<float> baseColor, float fresnel, float roughness,
             float metallic) {
            return context.createMetallicMaterial(
                {baseColor.at(0), baseColor.at(1), baseColor.at(2), baseColor.at(3)}, fresnel,
                roughness, metallic, 0.f);
          },
          py::arg("base_colorf"), py::arg("specular"), py::arg("roughness"), py::arg("metallic"))
      .def(
          "create_model_from_file",
          [](core::Context &context, std::string const &filename) {
            context.getResourceManager().CreateModelFromFile(filename);
          },
          py::arg("filename"))
      .def(
          "create_texture_from_file",
          [](core::Context &context, std::string const &filename, uint32_t mipmapLevels,
             std::string const &filter, std::string const &addressMode) {
            vk::Filter vkFilter;
            if (filter == "linear") {
              vkFilter = vk::Filter::eLinear;
            } else if (filter == "nearest") {
              vkFilter = vk::Filter::eNearest;
            } else {
              throw std::runtime_error("unknown filter: " + filter +
                                       ". should be one of linear or nearest.");
            }
            vk::SamplerAddressMode vkAddressMode;
            if (addressMode == "repeat") {
              vkAddressMode = vk::SamplerAddressMode::eRepeat;
            } else if (addressMode == "border") {
              vkAddressMode = vk::SamplerAddressMode::eClampToBorder;
            } else if (addressMode == "edge") {
              vkAddressMode = vk::SamplerAddressMode::eClampToEdge;
            } else {
              throw std::runtime_error("unknown address mode: " + filter +
                                       ". should be one of repeat, border, or edge.");
            }
            return context.getResourceManager().CreateTextureFromFile(
                filename, mipmapLevels, vkFilter, vkFilter, vkAddressMode, vkAddressMode);
          },
          py::arg("filename"), py::arg("mipmap_levels"), py::arg("filter") = "linear",
          py::arg("address_mode") = "repeat")
      .def(
          "create_capsule_mesh",
          [](core::Context &, float radius, float halfLength, int segments, int halfRings) {
            return resource::SVMesh::CreateCapsule(radius, halfLength, segments, halfRings);
          },
          py::arg("radius"), py::arg("half_length"), py::arg("segments") = 32,
          py::arg("half_rings") = 8)
      .def(
          "create_uvsphere_mesh",
          [](core::Context &, int segments, int rings) {
            return resource::SVMesh::CreateUVSphere(segments, rings);
          },
          py::arg("segments") = 32, py::arg("half_rings") = 16)
      .def("create_box_mesh", [](core::Context &) { return resource::SVMesh::CreateCube(); })
      .def(
          "create_cone_mesh",
          [](core::Context &, int segments) { return resource::SVMesh::CreateCone(segments); },
          py::arg("segments") = 32)
      .def(
          "create_model",
          [](core::Context &context, std::vector<std::shared_ptr<resource::SVMesh>> const &meshes,
             std::vector<std::shared_ptr<resource::SVMetallicMaterial>> const &materials) {
            std::vector<std::shared_ptr<resource::SVMaterial>> mats;
            mats.insert(mats.end(), materials.begin(), materials.end());
            return context.createModel(meshes, mats);
          },
          py::arg("meshes"), py::arg("materials"));

  PyMaterial
      // .def("set_textures", &resource::SVMetallicMaterial::setTextures)
      .def(
          "set_base_color",
          [](resource::SVMetallicMaterial &mat, py::array_t<float> color) {
            mat.setBaseColor({color.at(0), color.at(1), color.at(2), color.at(3)});
          },
          py::arg("rgba"))
      .def("set_specular", &resource::SVMetallicMaterial::setFresnel, py::arg("specular"))
      .def("set_metallic", &resource::SVMetallicMaterial::setMetallic, py::arg("metallic"))
      .def("set_roughness", &resource::SVMetallicMaterial::setRoughness, py::arg("roughness"));

  PyScene
      .def(
          "add_node",
          [](scene::Scene &scene, scene::Node *parent) {
            if (parent) {
              return &scene.addNode(*parent);
            } else {
              return &scene.addNode();
            }
          },
          py::return_value_policy::reference, py::arg("parent") = nullptr)
      .def(
          "add_object",
          [](scene::Scene &scene, std::shared_ptr<resource::SVModel> model, scene::Node *parent) {
            if (!model) {
              throw std::runtime_error("add object failed: model must not be None");
            }
            if (parent) {
              return &scene.addObject(*parent, model);
            } else {
              return &scene.addObject(model);
            }
          },
          py::return_value_policy::reference, py::arg("model"), py::arg("parent") = nullptr)
      .def("remove_node", &scene::Scene::removeNode, py::arg("node"));

  PySceneNode
      .def(
          "set_scale",
          [](scene::Node &node, py::array_t<float> scale) {
            node.setScale({scale.at(0), scale.at(1), scale.at(2)});
          },
          py::arg("scale"))
      .def(
          "set_position",
          [](scene::Node &node, py::array_t<float> position) {
            node.setPosition({position.at(0), position.at(1), position.at(2)});
          },
          py::arg("position"))
      .def(
          "set_rotation",
          [](scene::Node &node, py::array_t<float> rotation) {
            node.setRotation({rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3)});
          },
          py::arg("quat"));

  PySceneObject
      .def_property("shading_mode", &scene::Object::getShadingMode, &scene::Object::setShadingMode)
      .def_property("transparency", &scene::Object::getTransparency,
                    &scene::Object::getTransparency);
}
