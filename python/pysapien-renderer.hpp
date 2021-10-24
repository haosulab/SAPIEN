#pragma once

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

void buildRenderer(py::module &parent) {
  py::module m = parent.def_submodule("renderer");

  auto PyContext = py::class_<core::Context, std::shared_ptr<core::Context>>(m, "Context");
  auto PyScene = py::class_<scene::Scene>(m, "Scene");
  auto PySceneNode = py::class_<scene::Node>(m, "Node");
  auto PySceneObject = py::class_<scene::Object, scene::Node>(m, "Object");
  auto PyLineSetObject = py::class_<scene::LineObject, scene::Node>(m, "LineSetObject");
  auto PyPointSetObject = py::class_<scene::PointObject, scene::Node>(m, "PointSetObject");

  auto PyTexture =
      py::class_<resource::SVTexture, std::shared_ptr<resource::SVTexture>>(m, "Texture");
  auto PyCubemap =
      py::class_<resource::SVCubemap, std::shared_ptr<resource::SVCubemap>>(m, "Cubemap");
  auto PyMaterial =
      py::class_<resource::SVMetallicMaterial, std::shared_ptr<resource::SVMetallicMaterial>>(
          m, "Material");
  auto PyModel = py::class_<resource::SVModel, std::shared_ptr<resource::SVModel>>(m, "Model");
  auto PyShape = py::class_<resource::SVShape, std::shared_ptr<resource::SVShape>>(m, "Shape");
  auto PyMesh = py::class_<resource::SVMesh, std::shared_ptr<resource::SVMesh>>(m, "Mesh");
  auto PyPrimitiveSet =
      py::class_<resource::SVPrimitiveSet, std::shared_ptr<resource::SVPrimitiveSet>>(
          m, "PrimitiveSet");

  auto PyRenderer = py::class_<renderer::Renderer>(m, "Renderer");

  auto PyUIWidget = py::class_<ui::Widget, std::shared_ptr<ui::Widget>>(m, "UIWidget");
  auto PyUIWindow = py::class_<ui::Window, ui::Widget, std::shared_ptr<ui::Window>>(m, "UIWindow");
  auto PyUISameLine =
      py::class_<ui::SameLine, ui::Widget, std::shared_ptr<ui::SameLine>>(m, "UISameLine");
  auto PyUITreeNode =
      py::class_<ui::TreeNode, ui::Widget, std::shared_ptr<ui::TreeNode>>(m, "UITreeNode");
  auto PyUISection =
      py::class_<ui::Section, ui::Widget, std::shared_ptr<ui::Section>>(m, "UISection");

  auto PyUICheckbox =
      py::class_<ui::Checkbox, ui::Widget, std::shared_ptr<ui::Checkbox>>(m, "UICheckbox");
  auto PyUIButton = py::class_<ui::Button, ui::Widget, std::shared_ptr<ui::Button>>(m, "UIButton");
  auto PyUISelectable =
      py::class_<ui::Selectable, ui::Widget, std::shared_ptr<ui::Selectable>>(m, "UISelectable");
  auto PyUIOptions =
      py::class_<ui::Options, ui::Widget, std::shared_ptr<ui::Options>>(m, "UIOptions");

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
  auto PyUIGizmo = py::class_<ui::Gizmo, ui::Widget, std::shared_ptr<ui::Gizmo>>(m, "UIGizmo");

  PyUIWidget.def("remove", &ui::Widget::remove)
      .def("remove_children", &ui::Widget::removeChildren)
      .def("get_children", &ui::Widget::getChildren);
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

  PyUITreeNode.def(py::init<>())
      .def("Label", &ui::TreeNode::Label, py::arg("label"))
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

  PyUISelectable.def(py::init<>())
      .def("Label", &ui::Selectable::Label, py::arg("label"))
      .def("Selected", &ui::Selectable::Selected, py::arg("selected"))
      .def("Callback", &ui::Selectable::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::Selectable::getSelected);

  PyUICheckbox.def(py::init<>())
      .def("Label", &ui::Checkbox::Label, py::arg("label"))
      .def("Checked", &ui::Checkbox::Checked, py::arg("checked"))
      .def("Callback", &ui::Checkbox::Callback, py::arg("func"))
      .def_property_readonly("checked", &ui::Checkbox::get);
  PyUIOptions.def(py::init<>())
      .def("Style", &ui::Options::Style, py::arg("style"))
      .def("Label", &ui::Options::Label, py::arg("label"))
      .def("Index", &ui::Options::Index, py::arg("index"))
      .def("Items", &ui::Options::Items, py::arg("items"))
      .def("Callback", &ui::Options::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::Options::get)
      .def_property_readonly("index", &ui::Options::getIndex);

  PyUIDisplayText.def(py::init<>()).def("Text", &ui::DisplayText::Text, py::arg("text"));

  PyUIInputText.def(py::init<>())
      .def("Label", &ui::InputText::Label, py::arg("label"))
      .def("Value", &ui::InputText::Value, py::arg("value"))
      .def_property_readonly("value", &ui::InputText::get)
      .def("Size", &ui::InputText::Size, py::arg("size"))
      .def("ReadOnly", &ui::InputText::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputText::Callback, py::arg("func"));

  PyUIInputFloat.def(py::init<>())
      .def("Label", &ui::InputFloat::Label, py::arg("label"))
      .def(
          "Value", [](ui::InputFloat &input, float x) { return input.Value(x); }, py::arg("value"))
      .def_property_readonly("value", [](ui::InputFloat &input) { return input.get(); })
      .def("ReadOnly", &ui::InputFloat::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat::Callback, py::arg("func"));

  PyUIInputFloat2.def(py::init<>())
      .def("Label", &ui::InputFloat2::Label, py::arg("label"))
      .def(
          "Value",
          [](ui::InputFloat2 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputFloat2 &input) {
                               glm::vec2 v = input.get();
                               return py::array_t<float>(2u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputFloat2::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat2::Callback, py::arg("func"));

  PyUIInputFloat3.def(py::init<>())
      .def("Label", &ui::InputFloat3::Label, py::arg("label"))
      .def(
          "Value",
          [](ui::InputFloat3 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1), x.at(2)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputFloat3 &input) {
                               glm::vec3 v = input.get();
                               return py::array_t<float>(3u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputFloat3::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat3::Callback, py::arg("func"));

  PyUIInputFloat4.def(py::init<>())
      .def("Label", &ui::InputFloat4::Label, py::arg("label"))
      .def(
          "Value",
          [](ui::InputFloat4 &input, py::array_t<float> x) {
            return input.Value({x.at(0), x.at(1), x.at(2), x.at(3)});
          },
          py::arg("value"))
      .def_property_readonly("value",
                             [](ui::InputFloat4 &input) {
                               glm::vec4 v = input.get();
                               return py::array_t<float>(4u, &v[0]);
                             })
      .def("ReadOnly", &ui::InputFloat4::ReadOnly, py::arg("read_only"))
      .def("Callback", &ui::InputFloat4::Callback, py::arg("func"));

  PyUISliderFloat.def(py::init<>())
      .def("Width", &ui::SliderFloat::Width, py::arg("width"))
      .def("Label", &ui::SliderFloat::Label, py::arg("label"))
      .def("Min", &ui::SliderFloat::Min, py::arg("min"))
      .def("Max", &ui::SliderFloat::Max, py::arg("max"))
      .def("Value", &ui::SliderFloat::Value, py::arg("value"))
      .def("Callback", &ui::SliderFloat::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::SliderFloat::get);

  PyUISliderAngle.def(py::init<>())
      .def("Width", &ui::SliderAngle::Width, py::arg("width"))
      .def("Label", &ui::SliderAngle::Label, py::arg("label"))
      .def("Min", &ui::SliderAngle::Min, py::arg("min"))
      .def("Max", &ui::SliderAngle::Max, py::arg("max"))
      .def("Value", &ui::SliderAngle::Value, py::arg("value"))
      .def("Callback", &ui::SliderAngle::Callback, py::arg("func"))
      .def_property_readonly("value", &ui::SliderAngle::get);

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
             glm::mat4 p(proj.at(0, 0), proj.at(1, 0), proj.at(2, 0), proj.at(3, 0),
                         proj.at(0, 1), -proj.at(1, 1), -proj.at(2, 1), proj.at(3, 1),
                         proj.at(0, 2), proj.at(1, 2), proj.at(2, 2), proj.at(3, 2),
                         proj.at(0, 3), proj.at(1, 3), proj.at(2, 3), proj.at(3, 3));
             gizmo.setCameraParameters(v, p);
           });
  // end UI

  PyContext
      .def(
          "create_material",
          [](core::Context &context, py::array_t<float> emission, py::array_t<float> baseColor,
             float fresnel, float roughness, float metallic) {
            return context.createMetallicMaterial(
                {emission.at(0), emission.at(1), emission.at(2), emission.at(3)},
                {baseColor.at(0), baseColor.at(1), baseColor.at(2), baseColor.at(3)}, fresnel,
                roughness, metallic, 0.f);
          },
          py::arg("emission"), py::arg("base_color"), py::arg("specular"), py::arg("roughness"),
          py::arg("metallic"))
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
            return context.createModel(meshes, mats);
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
           py::arg("emission") = nullptr)
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
      .def("remove_node", &scene::Scene::removeNode, py::arg("node"));

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
      .def_property_readonly("model", &scene::Object::getModel);

  PyRenderer
      .def("set_custom_texture", &renderer::Renderer::setCustomTexture, py::arg("name"),
           py::arg("texture"))
      .def("set_custom_cubemap", &renderer::Renderer::setCustomCubemap, py::arg("name"),
           py::arg("texture"))
      .def("set_specialization_constant_int", &renderer::Renderer::setSpecializationConstantInt,
           py::arg("name"), py::arg("value"))
      .def("set_specialization_constant_float",
           &renderer::Renderer::setSpecializationConstantFloat, py::arg("name"), py::arg("value"));
}
