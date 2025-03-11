/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/scene/scene.h>
#include <svulkan2/ui/ui.h>

namespace py = pybind11;
using namespace svulkan2;

template <typename T> std::shared_ptr<T> widgetAppend(T &widget, py::args args) {
  if (args.size() == 0) {
    throw std::runtime_error("append must take 1 or more arguments");
  }
  std::shared_ptr<T> result;
  for (auto &arg : args) {
    result = widget.append(arg.cast<std::shared_ptr<ui::Widget>>());
  }
  return result;
}

template <typename T> using Setter = std::function<void(T)>;
template <typename T> using Getter = std::function<T()>;

template <typename T> Getter<T> bindReadonly(py::object obj, std::string name) {
  auto getter = [=]() { return obj.attr(name.c_str()).cast<T>(); };
  return getter;
}

template <typename T> std::tuple<Getter<T>, Setter<T>> bind(py::object obj, std::string name) {
  auto getter = [=]() { return obj.attr(name.c_str()).cast<T>(); };
  auto setter = [=](T value) { obj.attr(name.c_str()) = value; };
  return {getter, setter};
}

template <typename T> std::tuple<Getter<T>, Setter<T>> bind(py::list list, int index) {
  auto getter = [=]() { return list[index].cast<T>(); };
  auto setter = [=](T value) { list[index] = value; };
  return {getter, setter};
}

template <typename Widget, typename Val,
          std::shared_ptr<Widget> (Widget::*Func)(Getter<Val>, Setter<Val>)>
std::shared_ptr<Widget> bindObject(Widget &widget, py::object obj, std::string name) {
  auto [getter, setter] = bind<Val>(obj, name);
  return (widget.*Func)(getter, setter);
}

template <typename Widget, typename Val, std::shared_ptr<Widget> (Widget::*Func)(Getter<Val>)>
std::shared_ptr<Widget> bindObjectReadonly(Widget &widget, py::object obj, std::string name) {
  auto getter = bindReadonly<Val>(obj, name);
  return (widget.*Func)(getter);
}

template <typename Widget, typename Val,
          std::shared_ptr<Widget> (Widget::*Func)(Getter<Val>, Setter<Val>)>
std::shared_ptr<Widget> bindList(Widget &widget, py::object list, int index) {
  auto [getter, setter] = bind<Val>(list, index);
  return (widget.*Func)(getter, setter);
}

class PyKeyframe : public ui::Keyframe, public py::trampoline_self_life_support {
public:
  using ui::Keyframe::Keyframe;
  int frame() override { PYBIND11_OVERRIDE_PURE(int, ui::Keyframe, frame, ); }
};

class PyDuration : public ui::Duration, public py::trampoline_self_life_support {
public:
  using ui::Duration::Duration;
  std::shared_ptr<ui::Keyframe> keyframe0() override {
    PYBIND11_OVERRIDE_PURE(std::shared_ptr<ui::Keyframe>, ui::Duration, keyframe0, );
  }
  std::shared_ptr<ui::Keyframe> keyframe1() override {
    PYBIND11_OVERRIDE_PURE(std::shared_ptr<ui::Keyframe>, ui::Duration, keyframe1, );
  }
  std::string name() override { PYBIND11_OVERRIDE_PURE(std::string, ui::Duration, name, ); }
};

void init_sapien_renderer_internal(py::module &parent) {
  py::module m = parent.def_submodule("internal_renderer");

  auto PyContext = py::classh<core::Context>(m, "Context");
  auto PyScene = py::classh<scene::Scene>(m, "Scene");
  auto PySceneNode = py::classh<scene::Node>(m, "Node");
  auto PySceneObject = py::classh<scene::Object, scene::Node>(m, "Object");
  auto PyLineSetObject = py::classh<scene::LineObject, scene::Node>(m, "LineSetObject");
  auto PyPointSetObject = py::classh<scene::PointObject, scene::Node>(m, "PointSetObject");

  auto PyTexture = py::classh<resource::SVTexture>(m, "Texture");
  auto PyCubemap = py::classh<resource::SVCubemap>(m, "Cubemap");
  auto PyMaterial = py::classh<resource::SVMetallicMaterial>(m, "Material");
  auto PyModel = py::classh<resource::SVModel>(m, "Model");
  auto PyShape = py::classh<resource::SVShape>(m, "Shape");
  auto PyMesh = py::classh<resource::SVMesh>(m, "Mesh");

  auto PyPrimitiveSet = py::classh<resource::SVPrimitiveSet>(m, "PrimitiveSet");
  auto PyLineSet = py::classh<resource::SVPointSet, resource::SVPrimitiveSet>(m, "LineSet");
  auto PyPointSet = py::classh<resource::SVLineSet, resource::SVPrimitiveSet>(m, "PointSet");

  auto PyRenderer = py::classh<renderer::RendererBase>(m, "Renderer");

  auto PyUIWidget = py::classh<ui::Widget>(m, "UIWidget");
  auto PyUIWindow = py::classh<ui::Window, ui::Widget>(m, "UIWindow");

  auto PyUISameLine = py::classh<ui::SameLine, ui::Widget>(m, "UISameLine");
  auto PyUIConditional = py::classh<ui::Conditional, ui::Widget>(m, "UIConditional");

  auto PyUITreeNode = py::classh<ui::TreeNode, ui::Widget>(m, "UITreeNode");
  auto PyUISection = py::classh<ui::Section, ui::Widget>(m, "UISection");

  auto PyUICheckbox = py::classh<ui::Checkbox, ui::Widget>(m, "UICheckbox");
  auto PyUIButton = py::classh<ui::Button, ui::Widget>(m, "UIButton");
  auto PyUISelectable = py::classh<ui::Selectable, ui::Widget>(m, "UISelectable");
  auto PyUIOptions = py::classh<ui::Options, ui::Widget>(m, "UIOptions");

  auto PyUIDisplayText = py::classh<ui::DisplayText, ui::Widget>(m, "UIDisplayText");
  auto PyUIInputText = py::classh<ui::InputText, ui::Widget>(m, "UIInputText");
  auto PyUIInputTextMultiline =
      py::classh<ui::InputTextMultiline, ui::Widget>(m, "UIInputTextMultiline");

  auto PyUIInputFloat = py::classh<ui::InputFloat, ui::Widget>(m, "UIInputFloat");
  auto PyUIInputFloat2 = py::classh<ui::InputFloat2, ui::Widget>(m, "UIInputFloat2");
  auto PyUIInputFloat3 = py::classh<ui::InputFloat3, ui::Widget>(m, "UIInputFloat3");
  auto PyUIInputFloat4 = py::classh<ui::InputFloat4, ui::Widget>(m, "UIInputFloat4");

  auto PyUIInputInt = py::classh<ui::InputInt, ui::Widget>(m, "UIInputInt");
  auto PyUIInputInt2 = py::classh<ui::InputInt2, ui::Widget>(m, "UIInputInt2");
  auto PyUIInputInt3 = py::classh<ui::InputInt3, ui::Widget>(m, "UIInputInt3");
  auto PyUIInputInt4 = py::classh<ui::InputInt4, ui::Widget>(m, "UIInputInt4");

  auto PyUISliderFloat = py::classh<ui::SliderFloat, ui::Widget>(m, "UISliderFloat");
  auto PyUISliderAngle = py::classh<ui::SliderAngle, ui::Widget>(m, "UISliderAngle");
  auto PyUIGizmo = py::classh<ui::Gizmo, ui::Widget>(m, "UIGizmo");

  auto PyUIKeyframe = py::classh<ui::Keyframe, PyKeyframe>(m, "UIKeyframe");
  auto PyUIDuration = py::classh<ui::Duration, PyDuration>(m, "UIDuration");

  auto PyUIKeyframeEditor = py::classh<ui::KeyframeEditor, ui::Widget>(m, "UIKeyframeEditor");
  auto PyUIFileChooser = py::classh<ui::FileChooser, ui::Widget>(m, "UIFileChooser");
  auto PyUIPopup = py::classh<ui::Popup, ui::Widget>(m, "UIPopup");

  auto PyUIDummy = py::classh<ui::Dummy, ui::Widget>(m, "UIDummy");

  auto PyUIPicture = py::classh<ui::DisplayImage, ui::Widget>(m, "UIPicture");

  PyUIWidget.def("remove", &ui::Widget::remove)
      .def("remove_children", &ui::Widget::removeChildren)
      .def("get_children", &ui::Widget::getChildren);
  // UI
  PyUIWindow.def("Label", &ui::Window::Label, py::arg("label"))
      .def("Id", &ui::Window::Id, py::arg("id"))
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

  PyUISameLine.def(py::init<>())
      .def("append",
           [](ui::SameLine &node, py::args args) {
             if (args.size() == 0) {
               throw std::runtime_error("append must take 1 or more arguments");
             }
             std::shared_ptr<ui::SameLine> result;
             for (auto &arg : args) {
               auto widget = arg.cast<std::shared_ptr<ui::Widget>>();
               result = node.append(widget);
             }
             return result;
           })
      .def("Offset", &ui::SameLine::Offset, py::arg("offset"))
      .def("Spacing", &ui::SameLine::Spacing, py::arg("spacing"));

  PyUIConditional.def(py::init<>())
      .def("append",
           [](ui::Conditional &node, py::args args) {
             if (args.size() == 0) {
               throw std::runtime_error("append must take 1 or more arguments");
             }
             std::shared_ptr<ui::Conditional> result;
             for (auto &arg : args) {
               auto widget = arg.cast<std::shared_ptr<ui::Widget>>();
               result = node.append(widget);
             }
             return result;
           })
      .def("Bind", bindObjectReadonly<ui::Conditional, bool, &ui::Conditional::BindCondition>)
      .def("Bind", &ui::Conditional::BindCondition);

  PyUITreeNode.def(py::init<>())
      .def("Label", &ui::TreeNode::Label, py::arg("label"))
      .def("Id", &ui::TreeNode::Id, py::arg("id"))
      .def("append", [](ui::TreeNode &node, py::args args) {
        if (args.size() == 0) {
          throw std::runtime_error("append must take 1 or more arguments");
        }
        std::shared_ptr<ui::TreeNode> result;
        for (auto &arg : args) {
          auto widget = arg.cast<std::shared_ptr<ui::Widget>>();
          result = node.append(widget);
        }
        return result;
      });

  PyUISection.def(py::init<>())
      .def("Label", &ui::Section::Label, py::arg("label"))
      .def("Id", &ui::Section::Id, py::arg("id"))
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
      .def("Id", &ui::Button::Id, py::arg("id"))
      .def("Callback", &ui::Button::Callback, py::arg("func"))
      .def("Width", &ui::Button::Width, py::arg("width"));

  PyUISelectable.def(py::init<>())
      .def("Label", &ui::Selectable::Label, py::arg("label"))
      .def("Id", &ui::Selectable::Id, py::arg("id"))
      .def("Selected", &ui::Selectable::Selected, py::arg("selected"))
      .def("Callback", &ui::Selectable::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::Selectable::getSelected);

  PyUICheckbox.def(py::init<>())
      .def("Label", &ui::Checkbox::Label, py::arg("label"))
      .def("Id", &ui::Checkbox::Id, py::arg("id"))
      .def("Checked", &ui::Checkbox::Checked, py::arg("checked"))
      .def("Callback", &ui::Checkbox::Callback, py::arg("func"))
      .def_property_readonly("checked", &ui::Checkbox::get)
      .def("Bind", bindObject<ui::Checkbox, bool, &ui::Checkbox::BindChecked>)
      .def("Bind", bindList<ui::Checkbox, bool, &ui::Checkbox::BindChecked>);

  PyUIOptions.def(py::init<>())
      .def("Style", &ui::Options::Style, py::arg("style"))
      .def("Label", &ui::Options::Label, py::arg("label"))
      .def("Id", &ui::Options::Id, py::arg("id"))
      .def("Index", &ui::Options::Index, py::arg("index"))
      .def("Items", &ui::Options::Items, py::arg("items"))
      .def("Callback", &ui::Options::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::Options::get)
      .def_property_readonly("index", &ui::Options::getIndex)
      .def("BindIndex", bindObject<ui::Options, int, &ui::Options::BindIndex>)
      .def("BindItems",
           bindObjectReadonly<ui::Options, std::vector<std::string>, &ui::Options::BindItems>);

  PyUIDisplayText.def(py::init<>())
      .def("Text", &ui::DisplayText::Text, py::arg("text"))
      .def("Bind", bindObjectReadonly<ui::DisplayText, std::string, &ui::DisplayText::BindText>)
      .def("Bind", &ui::DisplayText::BindText);

  PyUIInputText.def(py::init<>())
      .def("Label", &ui::InputText::Label, py::arg("label"))
      .def("Id", &ui::InputText::Id, py::arg("id"))
      .def("Value", &ui::InputText::Value, py::arg("value"))
      .def_property_readonly("value", &ui::InputText::get)
      .def("Size", &ui::InputText::Size, py::arg("size"))
      .def("ReadOnly", &ui::InputText::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputText::Callback, py::arg("func"))
      .def("WidthRatio", &ui::InputText::WidthRatio, py::arg("width"));

  PyUIInputTextMultiline.def(py::init<>())
      .def("Label", &ui::InputTextMultiline::Label, py::arg("label"))
      .def("Id", &ui::InputTextMultiline::Id, py::arg("id"))
      .def("Value", &ui::InputTextMultiline::Value, py::arg("value"))
      .def_property_readonly("value", &ui::InputTextMultiline::get)
      .def("Size", &ui::InputTextMultiline::Size, py::arg("size"))
      .def("ReadOnly", &ui::InputTextMultiline::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputTextMultiline::Callback, py::arg("func"));

  PyUIInputFloat.def(py::init<>())
      .def("Label", &ui::InputFloat::Label, py::arg("label"))
      .def("Id", &ui::InputFloat::Id, py::arg("id"))
      .def(
          "Value", [](ui::InputFloat &input, float x) { return input.Value(x); }, py::arg("value"))
      .def_property_readonly("value", [](ui::InputFloat &input) { return input.get(); })
      .def("ReadOnly", &ui::InputFloat::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputFloat, float, &ui::InputFloat::BindValue>)
      .def("WidthRatio", &ui::InputFloat::WidthRatio, py::arg("width"));

  PyUIInputFloat2.def(py::init<>())
      .def("Label", &ui::InputFloat2::Label, py::arg("label"))
      .def("Id", &ui::InputFloat2::Id, py::arg("id"))
      .def(
          "Value",
          [](ui::InputFloat2 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputFloat2 &input) {
                               auto v = input.get();
                               return py::array_t<float>(2u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputFloat2::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat2::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputFloat2, std::array<float, 2>, &ui::InputFloat2::BindValue>)
      .def("WidthRatio", &ui::InputFloat2::WidthRatio, py::arg("width"));

  PyUIInputFloat3.def(py::init<>())
      .def("Label", &ui::InputFloat3::Label, py::arg("label"))
      .def("Id", &ui::InputFloat3::Id, py::arg("id"))
      .def(
          "Value",
          [](ui::InputFloat3 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1), x.at(2)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputFloat3 &input) {
                               auto v = input.get();
                               return py::array_t<float>(3u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputFloat3::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat3::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputFloat3, std::array<float, 3>, &ui::InputFloat3::BindValue>)
      .def("WidthRatio", &ui::InputFloat3::WidthRatio, py::arg("width"));

  PyUIInputFloat4.def(py::init<>())
      .def("Label", &ui::InputFloat4::Label, py::arg("label"))
      .def("Id", &ui::InputFloat4::Id, py::arg("id"))
      .def(
          "Value",
          [](ui::InputFloat4 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1), x.at(2), x.at(3)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputFloat4 &input) {
                               auto v = input.get();
                               return py::array_t<float>(4u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputFloat4::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat4::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputFloat4, std::array<float, 4>, &ui::InputFloat4::BindValue>)
      .def("WidthRatio", &ui::InputFloat4::WidthRatio, py::arg("width"));

  PyUIInputInt.def(py::init<>())
      .def("Label", &ui::InputInt::Label, py::arg("label"))
      .def("Id", &ui::InputInt::Id, py::arg("id"))
      .def(
          "Value", [](ui::InputInt &input, int x) { return input.Value(x); }, py::arg("value"))
      .def_property_readonly("value", [](ui::InputInt &input) { return input.get(); })
      .def("ReadOnly", &ui::InputInt::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputInt::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputInt, int, &ui::InputInt::BindValue>)
      .def("WidthRatio", &ui::InputInt::WidthRatio, py::arg("width"));

  PyUIInputInt2.def(py::init<>())
      .def("Label", &ui::InputInt2::Label, py::arg("label"))
      .def("Id", &ui::InputInt2::Id, py::arg("id"))
      .def(
          "Value",
          [](ui::InputInt2 &input, py::array_t<int> x) {
            return input.Value({x.at(0), x.at(1)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputInt2 &input) {
                               auto v = input.get();
                               return py::array_t<int>(2u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputInt2::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputInt2::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputInt2, std::array<int, 2>, &ui::InputInt2::BindValue>)
      .def("WidthRatio", &ui::InputInt2::WidthRatio, py::arg("width"));

  PyUIInputInt3.def(py::init<>())
      .def("Label", &ui::InputInt3::Label, py::arg("label"))
      .def("Id", &ui::InputInt3::Id, py::arg("id"))
      .def(
          "Value",
          [](ui::InputInt3 &input, py::array_t<int> x) {
            return input.Value({x.at(0), x.at(1), x.at(2)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputInt3 &input) {
                               auto v = input.get();
                               return py::array_t<int>(3u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputInt3::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputInt3::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputInt3, std::array<int, 3>, &ui::InputInt3::BindValue>)
      .def("WidthRatio", &ui::InputInt3::WidthRatio, py::arg("width"));

  PyUIInputInt4.def(py::init<>())
      .def("Label", &ui::InputInt4::Label, py::arg("label"))
      .def("Id", &ui::InputInt4::Id, py::arg("id"))
      .def(
          "Value",
          [](ui::InputInt4 &input, py::array_t<int> x) {
            return input.Value({x.at(0), x.at(1), x.at(2), x.at(3)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputInt4 &input) {
                               auto v = input.get();
                               return py::array_t<int>(4u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputInt4::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputInt4::Callback, py::arg("func"))
      .def("Bind", bindObject<ui::InputInt4, std::array<int, 4>, &ui::InputInt4::BindValue>)
      .def("WidthRatio", &ui::InputInt4::WidthRatio, py::arg("width"));

  PyUISliderFloat.def(py::init<>())
      .def("WidthRatio", &ui::SliderFloat::WidthRatio, py::arg("width"))
      .def("Label", &ui::SliderFloat::Label, py::arg("label"))
      .def("Id", &ui::SliderFloat::Id, py::arg("id"))
      .def("Min", &ui::SliderFloat::Min, py::arg("min"))
      .def("Max", &ui::SliderFloat::Max, py::arg("max"))
      .def("Value", &ui::SliderFloat::Value, py::arg("value"))
      .def("Callback", &ui::SliderFloat::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::SliderFloat::get)
      .def("Bind", bindObject<ui::SliderFloat, float, &ui::SliderFloat::BindValue>);

  PyUISliderAngle.def(py::init<>())
      .def("WidthRatio", &ui::SliderAngle::WidthRatio, py::arg("width"))
      .def("Label", &ui::SliderAngle::Label, py::arg("label"))
      .def("Id", &ui::SliderAngle::Id, py::arg("id"))
      .def("Min", &ui::SliderAngle::Min, py::arg("min"))
      .def("Max", &ui::SliderAngle::Max, py::arg("max"))
      .def("Value", &ui::SliderAngle::Value, py::arg("value"))
      .def("Callback", &ui::SliderAngle::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::SliderAngle::get)
      .def("Bind", bindObject<ui::SliderAngle, float, &ui::SliderAngle::BindValue>);

  PyUIGizmo.def(py::init<>())
      .def(
          "Matrix",
          [](ui::Gizmo &gizmo, py::array_t<float> matrix) {
            glm::mat4 m(matrix.at(0, 0), matrix.at(1, 0), matrix.at(2, 0), matrix.at(3, 0),
                        matrix.at(0, 1), matrix.at(1, 1), matrix.at(2, 1), matrix.at(3, 1),
                        matrix.at(0, 2), matrix.at(1, 2), matrix.at(2, 2), matrix.at(3, 2),
                        matrix.at(0, 3), matrix.at(1, 3), matrix.at(2, 3), matrix.at(3, 3));
            return gizmo.Matrix(m);
          },
          py::arg("matrix"))
      .def_property_readonly("matrix",
                             [](ui::Gizmo &gizmo) {
                               glm::mat4 mat = gizmo.getMatrix();
                               float arr[] = {mat[0][0], mat[1][0], mat[2][0], mat[3][0],
                                              mat[0][1], mat[1][1], mat[2][1], mat[3][1],
                                              mat[0][2], mat[1][2], mat[2][2], mat[3][2],
                                              mat[0][3], mat[1][3], mat[2][3], mat[3][3]};
                               return py::array_t<float>({4, 4}, arr);
                             })
      .def("CameraMatrices",
           [](ui::Gizmo &gizmo, py::array_t<float> view, py::array_t<float> proj) {
             glm::mat4 v(view.at(0, 0), view.at(1, 0), view.at(2, 0), view.at(3, 0), view.at(0, 1),
                         view.at(1, 1), view.at(2, 1), view.at(3, 1), view.at(0, 2), view.at(1, 2),
                         view.at(2, 2), view.at(3, 2), view.at(0, 3), view.at(1, 3), view.at(2, 3),
                         view.at(3, 3));
             glm::mat4 p(proj.at(0, 0), proj.at(1, 0), proj.at(2, 0), proj.at(3, 0), proj.at(0, 1),
                         -proj.at(1, 1), -proj.at(2, 1), proj.at(3, 1), proj.at(0, 2),
                         proj.at(1, 2), proj.at(2, 2), proj.at(3, 2), proj.at(0, 3), proj.at(1, 3),
                         proj.at(2, 3), proj.at(3, 3));
             gizmo.setCameraParameters(v, p);
           })
      .def("Bind", [](ui::Gizmo &gizmo, py::object obj, std::string name) {
        auto getter = [=]() {
          auto mat = obj.attr(name.c_str()).cast<py::array_t<float>>();
          return glm::mat4(mat.at(0, 0), mat.at(1, 0), mat.at(2, 0), mat.at(3, 0), mat.at(0, 1),
                           mat.at(1, 1), mat.at(2, 1), mat.at(3, 1), mat.at(0, 2), mat.at(1, 2),
                           mat.at(2, 2), mat.at(3, 2), mat.at(0, 3), mat.at(1, 3), mat.at(2, 3),
                           mat.at(3, 3));
        };
        auto setter = [=](glm::mat4 mat) {
          float arr[] = {mat[0][0], mat[1][0], mat[2][0], mat[3][0], mat[0][1], mat[1][1],
                         mat[2][1], mat[3][1], mat[0][2], mat[1][2], mat[2][2], mat[3][2],
                         mat[0][3], mat[1][3], mat[2][3], mat[3][3]};
          obj.attr(name.c_str()) = py::array_t<float>({4, 4}, arr);
        };
        return gizmo.BindMatrix(getter, setter);
      });

  PyUIKeyframe.def(py::init<>()).def("frame", &ui::Keyframe::frame);
  PyUIDuration.def(py::init<>())
      .def("keyframe0", &ui::Duration::keyframe0)
      .def("keyframe1", &ui::Duration::keyframe1)
      .def("name", &ui::Duration::name);

  PyUIKeyframeEditor
      .def(py::init([](float scale) { return ui::KeyframeEditor::Create(scale); }),
           py::arg("content_scale"))
      .def("BindTotalFrames",
           bindObject<ui::KeyframeEditor, int, &ui::KeyframeEditor::BindTotalFrames>,
           py::return_value_policy::reference)
      .def("BindCurrentFrame",
           bindObject<ui::KeyframeEditor, int, &ui::KeyframeEditor::BindCurrentFrame>,
           py::return_value_policy::reference)
      .def("AddDurationCallback", &ui::KeyframeEditor::AddDurationCallback, py::arg("func"),
           py::return_value_policy::reference)
      .def("AddKeyframeCallback", &ui::KeyframeEditor::AddKeyframeCallback, py::arg("func"),
           py::return_value_policy::reference)
      .def("MoveKeyframeCallback", &ui::KeyframeEditor::MoveKeyframeCallback, py::arg("func"),
           py::return_value_policy::reference)
      .def("DoubleClickKeyframeCallback", &ui::KeyframeEditor::DoubleClickKeyframeCallback,
           py::arg("func"), py::return_value_policy::reference)
      .def("DoubleClickDurationCallback", &ui::KeyframeEditor::DoubleClickDurationCallback,
           py::arg("func"), py::return_value_policy::reference)

      .def("add_keyframe", &ui::KeyframeEditor::addKeyframe, py::arg("keyframe"))
      .def("remove_keyframe", &ui::KeyframeEditor::removeKeyframe, py::arg("keyframe"))
      .def("add_duration", &ui::KeyframeEditor::addDuration, py::arg("duration"))
      .def("remove_duration", &ui::KeyframeEditor::removeDuration, py::arg("duration"))
      .def("set_state", &ui::KeyframeEditor::setState, py::arg("keyframes"), py::arg("durations"))
      .def("get_keyframes", &ui::KeyframeEditor::getKeyframes)
      .def("get_durations", &ui::KeyframeEditor::getDurations)
      .def("append", [](ui::KeyframeEditor &editor, py::args args) {
        if (args.size() == 0) {
          throw std::runtime_error("append must take 1 or more arguments");
        }
        std::shared_ptr<ui::KeyframeEditor> result;
        for (auto &arg : args) {
          auto widget = arg.cast<std::shared_ptr<ui::Widget>>();
          result = editor.append(widget);
        }
        return result;
      });

  PyUIFileChooser.def(py::init<>())
      .def("Label", &ui::FileChooser::Label, py::arg("label"), py::return_value_policy::reference)
      .def("Id", &ui::FileChooser::Id, py::arg("id"), py::return_value_policy::reference)
      .def("Title", &ui::FileChooser::Title, py::arg("title"), py::return_value_policy::reference)
      .def("Filter", &ui::FileChooser::Filter, py::arg("filter"),
           py::return_value_policy::reference)
      .def("Path", &ui::FileChooser::Path, py::arg("path"), py::return_value_policy::reference)
      .def("Callback", &ui::FileChooser::Callback, py::arg("func"),
           py::return_value_policy::reference)
      .def("open", &ui::FileChooser::open)
      .def("close", &ui::FileChooser::close);

  PyUIPopup.def(py::init<>())
      .def("append", widgetAppend<ui::Popup>, py::return_value_policy::reference)
      .def("Label", &ui::Popup::Label, py::arg("label"), py::return_value_policy::reference)
      .def("Id", &ui::Popup::Id, py::arg("id"), py::return_value_policy::reference)
      .def("EscCallback", &ui::Popup::EscCallback, py::arg("func"),
           py::return_value_policy::reference);

  PyUIDummy.def(py::init<>()).def("Width", &ui::Dummy::Width).def("Height", &ui::Dummy::Height);

  PyUIPicture.def(py::init<>())
      .def(
          "Size",
          [](ui::DisplayImage &image, float x, float y) {
            return image.Size({x, y});
          },
          py::arg("x"), py::arg("y"))
      .def(
          "Picture",
          [](ui::DisplayImage &image, renderer::RendererBase &renderer, std::string const &name) {
            return image.Image(renderer.getRenderImage(name));
          },
          py::arg("renderer"), py::arg("name"))
      .def("Clear", &ui::DisplayImage::Clear);

  // end UI

  PyContext
      .def(
          "create_material",
          [](core::Context &context, py::array_t<float> emission, py::array_t<float> baseColor,
             float fresnel, float roughness, float metallic, float transmission, float ior) {
            return context.getResourceManager()->createMetallicMaterial(
                {emission.at(0), emission.at(1), emission.at(2), emission.at(3)},
                {baseColor.at(0), baseColor.at(1), baseColor.at(2), baseColor.at(3)}, fresnel,
                roughness, metallic, transmission, ior);
          },
          py::arg("emission"), py::arg("base_color"), py::arg("specular"), py::arg("roughness"),
          py::arg("metallic"), py::arg("transmission") = 0.f, py::arg("ior") = 1.01f)
      .def(
          "create_model_from_file",
          [](core::Context &context, std::string const &filename) {
            return context.getResourceManager()->CreateModelFromFile(filename);
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
            return context.getResourceManager()->CreateTextureFromFile(
                filename, mipmapLevels, vkFilter, vkFilter, vkAddressMode, vkAddressMode);
          },
          py::arg("filename"), py::arg("mipmap_levels"), py::arg("filter") = "linear",
          py::arg("address_mode") = "repeat")
      .def(
          "create_cubemap_from_files",
          [](core::Context &context, std::array<std::string, 6> const &filenames,
             uint32_t mipmapLevels) {
            return context.getResourceManager()->CreateCubemapFromFiles(filenames, mipmapLevels);
          },
          "Load cube map, its mipmaps are generated based on roughness, details see "
          "https://learnopengl.com/PBR/IBL/Specular-IBL",
          py::arg("filenames"), py::arg("mipmap_levels"))
      .def(
          "create_brdf_lut",
          [](std::shared_ptr<core::Context> context, uint32_t size) {
            return context->getResourceManager()->generateBRDFLUT(size);
          },
          "Generate BRDF LUT texture, see https://learnopengl.com/PBR/IBL/Specular-IBL",
          py::arg("size") = 128)
      .def(
          "create_capsule_mesh",
          [](core::Context &, float radius, float halfLength, int segments, int halfRings) {
            return resource::SVMesh::CreateCapsule(radius, halfLength, segments, halfRings);
          },
          py::arg("radius"), py::arg("half_length"), py::arg("segments") = 32,
          py::arg("half_rings") = 8)
      .def(
          "create_cylinder_mesh",
          [](core::Context &, int segments) { return resource::SVMesh::CreateCylinder(segments); },
          py::arg("segments") = 32)
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
          "create_mesh_from_array",
          [](core::Context &, const py::array_t<float> &vertices,
             const py::array_t<uint32_t> &indices, const py::array_t<float> &normals,
             const py::array_t<float> &uvs) {
            auto mesh = svulkan2::resource::SVMesh::Create(
                std::vector<float>(vertices.data(), vertices.data() + vertices.size()),
                std::vector<uint32_t>(indices.data(), indices.data() + indices.size()));
            if (normals.size() != 0) {
              if (normals.size() != vertices.size()) {
                throw std::runtime_error(
                    "create mesh fail: the size of normals does not match with vertices");
              }
              mesh->setVertexAttribute(
                  "normal", std::vector<float>(normals.data(), normals.data() + normals.size()));
            }
            if (uvs.size() != 0) {
              if (uvs.size() != vertices.size()) {
                throw std::runtime_error(
                    "create mesh fail: the size of uvs does not match with vertices");
              }
              mesh->setVertexAttribute("uv",
                                       std::vector<float>(uvs.data(), uvs.data() + uvs.size()));
            }
            return mesh;
          },
          py::arg("vertices"), py::arg("indices"), py::arg("normals") = py::array_t<float>(),
          py::arg("uvs") = py::array_t<float>())
      .def(
          "create_model",
          [](core::Context &context, std::vector<std::shared_ptr<resource::SVMesh>> const &meshes,
             std::vector<std::shared_ptr<resource::SVMetallicMaterial>> const &materials) {
            std::vector<std::shared_ptr<resource::SVMaterial>> mats;
            mats.insert(mats.end(), materials.begin(), materials.end());
            return context.getResourceManager()->createModel(meshes, mats);
          },
          py::arg("meshes"), py::arg("materials"))
      .def(
          "create_line_set",
          [](core::Context &, const py::array_t<float> &vertices,
             const py::array_t<float> &colors) {
            auto lineset = std::make_shared<resource::SVLineSet>();
            std::vector<float> vs(vertices.data(), vertices.data() + vertices.size());
            std::vector<float> cs(colors.data(), colors.data() + colors.size());
            lineset->setVertexAttribute("position", vs);
            lineset->setVertexAttribute("color", cs);
            return lineset;
          },
          py::arg("vertices"), py::arg("colors"))
      .def(
          "create_point_set",
          [](core::Context &, const py::array_t<float> &vertices,
             const py::array_t<float> &colors) {
            auto pointset = std::make_shared<resource::SVPointSet>();
            std::vector<float> vs(vertices.data(), vertices.data() + vertices.size());
            std::vector<float> cs(colors.data(), colors.data() + colors.size());
            pointset->setVertexAttribute("position", vs);
            pointset->setVertexAttribute("color", cs);
            return pointset;
          },
          py::arg("vertices"), py::arg("colors"));

  PyMaterial
      .def("set_textures", &resource::SVMetallicMaterial::setTextures,
           py::arg("base_color") = nullptr, py::arg("roughness") = nullptr,
           py::arg("normal") = nullptr, py::arg("metallic") = nullptr,
           py::arg("emission") = nullptr, py::arg("transmission") = nullptr)
      .def(
          "set_base_color",
          [](resource::SVMetallicMaterial &mat, py::array_t<float> color) {
            mat.setBaseColor({color.at(0), color.at(1), color.at(2), color.at(3)});
          },
          py::arg("rgba"))
      .def("set_specular", &resource::SVMetallicMaterial::setFresnel, py::arg("specular"))
      .def("set_metallic", &resource::SVMetallicMaterial::setMetallic, py::arg("metallic"))
      .def("set_roughness", &resource::SVMetallicMaterial::setRoughness, py::arg("roughness"))
      .def("set_emission", &resource::SVMetallicMaterial::setRoughness, py::arg("emission"))
      .def("set_transmission", &resource::SVMetallicMaterial::setRoughness,
           py::arg("transmission"));

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
      .def(
          "add_line_set",
          [](scene::Scene &scene, std::shared_ptr<resource::SVLineSet> lineset,
             scene::Node *parent) {
            if (!lineset) {
              throw std::runtime_error("add line set failed: it must not be None");
            }
            if (parent) {
              return &scene.addLineObject(*parent, lineset);
            } else {
              return &scene.addLineObject(lineset);
            }
          },
          py::return_value_policy::reference, py::arg("line_set"), py::arg("parent") = nullptr)
      .def(
          "add_point_set",
          [](scene::Scene &scene, std::shared_ptr<resource::SVPointSet> pointset,
             scene::Node *parent) {
            if (!pointset) {
              throw std::runtime_error("add point set failed: it must not be None");
            }
            if (parent) {
              return &scene.addPointObject(*parent, pointset);
            } else {
              return &scene.addPointObject(pointset);
            }
          },
          py::return_value_policy::reference, py::arg("point_set"), py::arg("parent") = nullptr)
      .def("remove_node", &scene::Scene::removeNode, py::arg("node"))
      .def("set_ambient_light",
           [](scene::Scene &s, py::array_t<float> color) {
             s.setAmbientLight({color.at(0), color.at(1), color.at(2), 1.f});
           })
      .def("set_cubemap", &scene::Scene::setEnvironmentMap)

      .def("force_update", [](scene::Scene &s) { s.updateModelMatrices(); })
      .def("force_rebuild", &scene::Scene::updateVersion);

  PySceneNode
      .def(
          "set_scale",
          [](scene::Node &node, py::array_t<float> scale) {
            node.setScale({scale.at(0), scale.at(1), scale.at(2)});
          },
          py::arg("scale"))
      .def_property_readonly("scale",
                             [](scene::Node &node) {
                               auto scale = node.getScale();
                               return py::array_t<float>(3, &scale.x);
                             })
      .def(
          "set_position",
          [](scene::Node &node, py::array_t<float> position) {
            node.setPosition({position.at(0), position.at(1), position.at(2)});
          },
          py::arg("position"))
      .def_property_readonly("position",
                             [](scene::Node &node) {
                               auto pos = node.getPosition();
                               return py::array_t<float>(3, &pos.x);
                             })
      .def(
          "set_rotation",
          [](scene::Node &node, py::array_t<float> rotation) {
            node.setRotation({rotation.at(0), rotation.at(1), rotation.at(2), rotation.at(3)});
          },
          py::arg("quat"))
      .def_property_readonly("rotation",
                             [](scene::Node &node) {
                               auto quat = node.getRotation();
                               std::vector<float> q = {quat.w, quat.x, quat.y, quat.z};
                               return py::array_t<float>(4, q.data());
                             })
      .def_property_readonly("children", &scene::Node::getChildren,
                             py::return_value_policy::reference);

  PySceneObject
      .def_property("shading_mode", &scene::Object::getShadingMode, &scene::Object::setShadingMode)
      .def_property("transparency", &scene::Object::getTransparency,
                    &scene::Object::setTransparency)
      .def_property("cast_shadow", &scene::Object::getCastShadow, &scene::Object::setCastShadow)
      .def_property_readonly("model", &scene::Object::getModel)
      .def("get_segmentation",
           [](scene::Object &obj) {
             auto seg = obj.getSegmentation();
             return py::array_t<uint32_t>(4, &seg[0]);
           })
      .def("set_segmentation", [](scene::Object &obj, py::array_t<uint32_t> seg) {
        obj.setSegmentation({seg.at(0), seg.at(1), seg.at(2), seg.at(3)});
      });

  PyLineSetObject.def_property("line_width", &scene::LineObject::getLineWidth,
                               &scene::LineObject::setLineWidth);

  PyRenderer
      .def("set_custom_texture", &renderer::RendererBase::setCustomTexture, py::arg("name"),
           py::arg("texture"))
      .def("set_custom_cubemap", &renderer::RendererBase::setCustomCubemap, py::arg("name"),
           py::arg("texture"))
      .def(
          "set_custom_property",
          [](renderer::RendererBase &r, std::string name, float v) {
            r.setCustomProperty(name, v);
          },
          py::arg("name"), py::arg("value"))
      .def(
          "set_custom_property",
          [](renderer::RendererBase &r, std::string name, int v) { r.setCustomProperty(name, v); },
          py::arg("name"), py::arg("value"));
}
