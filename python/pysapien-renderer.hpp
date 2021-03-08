#pragma once
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <svulkan2/core/context.h>
#include <svulkan2/scene/scene.h>
#include <svulkan2/ui/ui.h>

namespace py = pybind11;

using namespace svulkan2;

void buildRenderer(py::module &parent) {
  py::module m = parent.def_submodule("renderer");

  auto PyContext = py::class_<core::Context>(m, "Context");
  auto PySceneNode = py::class_<scene::Node>(m, "Node");
  auto PyMaterial =
      py::class_<resource::SVMetallicMaterial, std::shared_ptr<resource::SVMetallicMaterial>>(
          m, "Material");
  auto PyModel = py::class_<resource::SVModel, std::shared_ptr<resource::SVModel>>(m, "Model");
  auto PyShape = py::class_<resource::SVShape, std::shared_ptr<resource::SVShape>>(m, "Shape");
  auto PyMesh = py::class_<resource::SVMesh, std::shared_ptr<resource::SVMesh>>(m, "Mesh");
  auto PyTexture =
      py::class_<resource::SVTexture, std::shared_ptr<resource::SVTexture>>(m, "Texture");

  auto PyUIWidget = py::class_<ui::Widget, std::shared_ptr<ui::Widget>>(m, "UIWidget");
  auto PyUIWindow = py::class_<ui::Window, ui::Widget, std::shared_ptr<ui::Window>>(m, "UIWindow");
  auto PyUICheckbox =
      py::class_<ui::Checkbox, ui::Widget, std::shared_ptr<ui::Checkbox>>(m, "UICheckbox");
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

  PyUIWidget.def("remove", &ui::Widget::remove);
  // UI
  PyUIWindow.def("Label", &ui::Window::Label, py::arg("Label"))
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

  PyUICheckbox.def(py::init<>())
      .def("Label", &ui::Checkbox::Label, py::arg("label"))
      .def("Checked", &ui::Checkbox::Checked, py::arg("checked"))
      .def("Callback", &ui::Checkbox::Callback, py::arg("func"))
      .def_property_readonly("checked", &ui::Checkbox::get);
  PyUIRadioButtonGroup.def(py::init<>())
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
      .def("create_material",
           [](core::Context &context, py::array_t<float> baseColor, float fresnel, float roughness,
              float metallic) {
             return context.createMetallicMaterial(
                 {baseColor.at(0), baseColor.at(1), baseColor.at(2), baseColor.at(3)}, fresnel,
                 roughness, metallic, 0.f);
           })
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
          py::arg("address_mode") = "repeat");
}
