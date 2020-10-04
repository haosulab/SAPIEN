#include "hud_object_window.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_articulation_base.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "articulation/sapien_kinematic_joint.h"
#include "articulation/sapien_link.h"
#include "sapien_actor.h"
#include "sapien_actor_base.h"
#include "sapien_contact.h"
#include "sapien_drive.h"
#include "sapien_scene.h"

#include "hud_common.h"

namespace sapien {
namespace Renderer {

void HudSettings::draw(SScene *scene) {
  if (ImGui::CollapsingHeader("Settings")) {
    auto flags = scene->getPxScene()->getFlags();
    bool b = flags & PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
    ImGui::Checkbox("Enhanced determinism", &b);
    b = flags & PxSceneFlag::eENABLE_PCM;
    ImGui::Checkbox("PCM(persistent contact manifold)", &b);
    b = flags & PxSceneFlag::eENABLE_CCD;
    ImGui::Checkbox("CCD(continuous collision detection)", &b);
    b = flags & PxSceneFlag::eENABLE_STABILIZATION;
    ImGui::Checkbox("Stabilization", &b);
    b = flags & PxSceneFlag::eENABLE_AVERAGE_POINT;
    ImGui::Checkbox("Average point", &b);
    b = flags & PxSceneFlag::eENABLE_GPU_DYNAMICS;
    ImGui::Checkbox("GPU dynamics", &b);
    b = flags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION;
    ImGui::Checkbox("Friction in every solver iteration", &b);
    b = flags & PxSceneFlag::eADAPTIVE_FORCE;
    ImGui::Checkbox("Adaptive force", &b);

    ImGui::Text("Contact offset: %.4g", scene->getDefaultContactOffset());
    ImGui::Text("Bounce threshold: %.4g", scene->getPxScene()->getBounceThresholdVelocity());
    ImGui::Text("Sleep threshold: %.4g", scene->getDefaultSleepThreshold());
    ImGui::Text("Solver iterations: %d", scene->getDefaultSolverIterations());
    ImGui::Text("Solver velocity iterations: %d",
                scene->getDefaultSolverVelocityIterations());
  }
}

void HudActor::draw(SActorBase *actor) {
  if (ImGui::CollapsingHeader("Actor", ImGuiTreeNodeFlags_DefaultOpen)) {
    // name
    ImGui::Text("Name: %s", actor->getName().c_str());

    {
      ImGui::Text("Display Opacity");
      float visibility = actor->getDisplayVisibility();
      if (ImGui::SliderFloat("##Opacity", &visibility, 0.f, 1.f)) {
        actor->setDisplayVisibility(visibility);
      }
      if (ImGui::Button("Make everything transparent")) {
        for (auto a : actor->getScene()->getAllActors()) {
          a->setDisplayVisibility(0.5);
        }
        for (auto a : actor->getScene()->getAllArticulations()) {
          for (auto b : a->getBaseLinks()) {
            b->setDisplayVisibility(0.5);
          }
        }
      }
      if (ImGui::Button("Make everything normal")) {
        for (auto a : actor->getScene()->getAllActors()) {
          a->setDisplayVisibility(1);
        }
        for (auto a : actor->getScene()->getAllArticulations()) {
          for (auto b : a->getBaseLinks()) {
            b->setDisplayVisibility(1);
          }
        }
      }
    }

    // type
    switch (actor->getType()) {
    case EActorType::STATIC:
      ImGui::Text("Type: static");
      break;
    case EActorType::KINEMATIC:
      ImGui::Text("Type: kinematic");
      break;
    case EActorType::DYNAMIC:
      ImGui::Text("Type: dynamic");
      break;
    case EActorType::ARTICULATION_LINK:
      ImGui::Text("Type: dynamic link");
      break;
    case EActorType::KINEMATIC_ARTICULATION_LINK:
      ImGui::Text("Type: kinematic link");
      break;
    };
    PrintPose(actor->getPose());
    ImGui::Text("col1: #%08x, col2: #%08x", actor->getCollisionGroup1(),
                actor->getCollisionGroup2());
    ImGui::Text("col3: #%08x", actor->getCollisionGroup3());

    static bool renderCollision;
    renderCollision = actor->isRenderingCollision();
    if (ImGui::Checkbox("Collision Shape", &renderCollision)) {
      actor->renderCollisionBodies(renderCollision);
    }

    auto pxActor = actor->getPxActor();
    if (actor->getType() != EActorType::STATIC) {
      auto body = static_cast<PxRigidBody *>(pxActor);
      ImGui::Text("Mass: %.4g", body->getMass());
      auto inertia = body->getMassSpaceInertiaTensor();
      ImGui::Text("Inertia: %.4g %.4g %.4g", inertia.x, inertia.y, inertia.z);
    }
    ImGui::Checkbox("Center of Mass", &mShowCenterOfMass);

    if (actor->getType() == EActorType::DYNAMIC) {
      bool b = static_cast<PxRigidDynamic *>(pxActor)->isSleeping();
      ImGui::Checkbox("Sleeping", &b);
    } else if (actor->getType() == EActorType::ARTICULATION_LINK) {
      bool b = static_cast<PxArticulationLink *>(pxActor)->getArticulation().isSleeping();
      ImGui::Checkbox("Sleeping", &b);
    }

    if (ImGui::TreeNode("Shapes")) {
      std::vector<PxShape *> shapes(pxActor->getNbShapes());
      pxActor->getShapes(shapes.data(), shapes.size());
      int primitives = 0;
      int meshes = 0;
      PxReal minDynamicFriction = 100;
      PxReal maxDynamicFriction = -1;
      PxReal minStaticFriction = 100;
      PxReal maxStaticFriction = -1;
      PxReal minRestitution = 100;
      PxReal maxRestitution = -1;

      PxReal minPatchRadius = 100000.f;
      PxReal maxPatchRadius = -1.f;

      PxReal minMinPatchRadius = 100000.f;
      PxReal maxMinPatchRadius = -1.f;

      for (auto s : shapes) {
        if (s->getGeometryType() == PxGeometryType::eCONVEXMESH) {
          meshes += 1;
        } else {
          primitives += 1;
        }
        std::vector<PxMaterial *> mats(s->getNbMaterials());
        s->getMaterials(mats.data(), s->getNbMaterials());
        for (auto m : mats) {
          PxReal sf = m->getStaticFriction();
          minStaticFriction = std::min(minStaticFriction, sf);
          maxStaticFriction = std::max(maxStaticFriction, sf);
          PxReal df = m->getDynamicFriction();
          minDynamicFriction = std::min(minDynamicFriction, df);
          maxDynamicFriction = std::max(maxDynamicFriction, df);
          PxReal r = m->getRestitution();
          minRestitution = std::min(minRestitution, r);
          maxRestitution = std::max(maxRestitution, r);
        }
        PxReal p = s->getTorsionalPatchRadius();
        minPatchRadius = std::min(minPatchRadius, p);
        maxPatchRadius = std::max(maxPatchRadius, p);
        p = s->getMinTorsionalPatchRadius();
        minMinPatchRadius = std::min(minMinPatchRadius, p);
        maxMinPatchRadius = std::max(maxMinPatchRadius, p);
      }
      ImGui::Text("Primitive Count: %d", primitives);
      ImGui::Text("Convex Mesh Count: %d", meshes);
      if (maxStaticFriction >= 0) {
        ImGui::Text("Static friction: %.4g - %.4g", minStaticFriction, maxStaticFriction);
        ImGui::Text("Dynamic friction: %.4g - %.4g", minDynamicFriction, maxDynamicFriction);
        ImGui::Text("Restitution : %.4g - %.4g", minRestitution, maxRestitution);
        ImGui::Text("Patch radius : %.4g - %.4g", minPatchRadius, maxPatchRadius);
        ImGui::Text("Min patch radius : %.4g - %.4g", minMinPatchRadius, maxMinPatchRadius);
      } else {
        ImGui::Text("No Physical Material");
      }
      ImGui::TreePop();
    }

    if (actor->getDrives().size()) {
      auto drives = actor->getDrives();
      if (ImGui::TreeNode("Drives")) {
        for (size_t i = 0; i < drives.size(); ++i) {
          ImGui::Text("Drive %ld", i + 1);
          if (drives[i]->getActor2() == actor) {
            if (drives[i]->getActor1()) {
              ImGui::Text("Drive towards pose relative to actor [%s]",
                          drives[i]->getActor1()->getName().c_str());
            } else {
              ImGui::Text("Drive towards absolute pose");
            }
          } else {
            if (drives[i]->getActor2()) {
              ImGui::Text("Drive other actor [%s] towards pose relative to this actor.",
                          drives[i]->getActor1()->getName().c_str());
            } else {
              ImGui::Text("This drive is created by specifying world frame as the second actor, "
                          "for "
                          "best performance, consider using world frame as the first actor");
            }
          }
          ImGui::NewLine();

          {
            // actor 1
            if (drives[i]->getActor1() == actor) {
              ImGui::Text("Actor 1: this actor");
            } else {
              ImGui::Text("Actor 1: %s", drives[i]->getActor1()
                                             ? drives[i]->getActor1()->getName().c_str()
                                             : "world frame");
            }
            auto pose1 = drives[i]->getLocalPose1();
            ImGui::Text("Drive attached at");
            ImGui::Text("Position: %.4g %.4g %.4g", pose1.p.x, pose1.p.y, pose1.p.z);
            glm::vec3 angles1 =
                glm::eulerAngles(glm::quat(pose1.q.w, pose1.q.x, pose1.q.y, pose1.q.z)) /
                glm::pi<float>() * 180.f;
            ImGui::Text("Euler (degree): %.4g %.4g %.4g", angles1.x, angles1.y, angles1.z);
            ImGui::Text("Quat (wxyz): %.4g %.4g %.4g %.4g", pose1.q.w, pose1.q.x, pose1.q.y,
                        pose1.q.z);
          }
          ImGui::NewLine();

          {
            // actor 2
            if (drives[i]->getActor2() == actor) {
              ImGui::Text("Actor 2: this actor");
            } else {
              ImGui::Text("Actor 2: %s", drives[i]->getActor2()
                                             ? drives[i]->getActor2()->getName().c_str()
                                             : "world frame");
            }
            auto pose2 = drives[i]->getLocalPose2();
            ImGui::Text("Drive attached at");
            ImGui::Text("Position: %.4g %.4g %.4g", pose2.p.x, pose2.p.y, pose2.p.z);
            glm::vec3 angles2 =
                glm::eulerAngles(glm::quat(pose2.q.w, pose2.q.x, pose2.q.y, pose2.q.z)) /
                glm::pi<float>() * 280.f;
            ImGui::Text("Euler (degree): %.4g %.4g %.4g", angles2.x, angles2.y, angles2.z);
            ImGui::Text("Quat (wxyz): %.4g %.4g %.4g %.4g", pose2.q.w, pose2.q.x, pose2.q.y,
                        pose2.q.z);
          }
          ImGui::NewLine();

          {
            auto target = drives[i]->getTarget();
            auto [v, w] = drives[i]->getTargetVelocity();

            ImGui::Text("Drive target");
            ImGui::Text("Position: %.4g %.4g %.4g", target.p.x, target.p.y, target.p.z);
            glm::vec3 angles =
                glm::eulerAngles(glm::quat(target.q.w, target.q.x, target.q.y, target.q.z)) /
                glm::pi<float>() * 180.f;
            ImGui::Text("Euler (degree): %.4g %.4g %.4g", angles.x, angles.y, angles.z);
            ImGui::Text("Quat (wxyz): %.4g %.4g %.4g %.4g", target.q.w, target.q.x, target.q.y,
                        target.q.z);
            ImGui::Text("Linear Velocity: %.4g %.4g %.4g", v.x, v.y, v.z);
            ImGui::Text("Angular Velocity: %.4g %.4g %.4g", w.x, w.y, w.z);

            if (ImGui::Button(("Remove Drive##" + std::to_string(i)).c_str())) {
              drives[i]->destroy();
            }
            ImGui::Text("Caution: Accessing a removed drive");
            ImGui::Text("will cause crash");
          }
          ImGui::NewLine();
        }
        ImGui::TreePop();
      }
    }
  }
}

void HudArticulation::draw(SArticulationBase *articulation, SActorBase *actor) {
  mSelect = false;
  if (ImGui::CollapsingHeader("Articulation")) {
    struct JointGuiModel {
      std::string name;
      std::array<float, 2> limits;
      float value;
    };
    std::vector<JointGuiModel> jointValues;
    jointValues.resize(articulation->dof());
    uint32_t n = 0;
    auto qpos = articulation->getQpos();
    for (auto j : articulation->getBaseJoints()) {
      auto limits = j->getLimits();
      for (uint32_t i = 0; i < j->getDof(); ++i) {
        jointValues[n].name = j->getName();
        jointValues[n].limits = limits[i];
        jointValues[n].value = qpos[n];
        ++n;
      }
    }

    if (ImGui::Button("Show collisions")) {
      for (auto l : articulation->getBaseLinks()) {
        l->renderCollisionBodies(true);
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Hide collisions")) {
      for (auto l : articulation->getBaseLinks()) {
        l->renderCollisionBodies(false);
      }
    }
    ImGui::Text("name: %s", articulation->getName().c_str());
    ImGui::Text("dof: %ld", jointValues.size());
    if (articulation->getType() == EArticulationType::DYNAMIC) {
      ImGui::Text("type: Dynamic");
    } else {
      ImGui::Text("type: Kinematic");
    }

    if (ImGui::TreeNode("Joints")) {
      static bool articulationDetails;
      ImGui::Checkbox("Details", &articulationDetails);

      std::vector<SJoint *> activeJoints;
      if (articulation->getType() == EArticulationType::DYNAMIC) {
        auto a = static_cast<sapien::SArticulation *>(articulation);
        auto js = a->getSJoints();
        for (auto j : js) {
          if (j->getDof()) {
            activeJoints.push_back(j);
          }
        }
      }

      int i = 0;
      for (auto &joint : jointValues) {
        ImGui::Text("joint: %s", joint.name.c_str());
        if (ImGui::SliderFloat(("##" + std::to_string(i)).c_str(), &joint.value,
                               std::max(joint.limits[0], -10.f),
                               std::min(joint.limits[1], 10.f))) {
          std::vector<PxReal> v;
          for (auto j : jointValues) {
            v.push_back(j.value);
          }
          articulation->setQpos(v);
        }
        if (articulationDetails && activeJoints.size()) {
          auto j = activeJoints[i];
          float friction = j->getFriction();
          float stiffness = j->getDriveStiffness();
          float damping = j->getDriveDamping();
          float maxForce = j->getDriveForceLimit();
          float target = j->getDriveTarget();
          float vtarget = j->getDriveVelocityTarget();
          ImGui::Text("Friction: %.4g", friction);
          ImGui::Text("Damping: %.4g", damping);
          ImGui::Text("Stiffness: %.4g", stiffness);
          if (maxForce > 1e6) {
            ImGui::Text("Max Force: >1e6");
          } else {
            ImGui::Text("Max Force: %.4g", maxForce);
          }
          if (stiffness > 0) {
            ImGui::Text("Drive Position Target: %.4g", target);
            ImGui::Text("Drive Velocity Target: %.4g", vtarget);
          }
          ImGui::NewLine();
        }
        ++i;
      }
      ImGui::TreePop();
    }

    // show links
    if (ImGui::TreeNode("Link Tree")) {
      auto links = articulation->getBaseLinks();
      auto joints = articulation->getBaseJoints();

      struct LinkNode {
        uint32_t parent;
        uint32_t index;
        std::vector<uint32_t> children;
      };
      std::vector<LinkNode> nodes(links.size());
      uint32_t root = joints.size();
      for (uint32_t i = 0; i < joints.size(); ++i) {
        auto p = joints[i]->getParentLink();
        nodes[i].index = i;
        if (p) {
          nodes[i].parent = p->getIndex();
          nodes[p->getIndex()].children.push_back(i);
        } else {
          root = i;
        }
      }

      std::vector<uint32_t> stack;
      std::vector<uint32_t> indents;
      stack.push_back(root);
      indents.push_back(0);

      while (!stack.empty()) {
        uint32_t idx = stack.back();
        uint32_t indent = indents.back();
        stack.pop_back();
        indents.pop_back();

        if (links[idx] == actor) {
          ImGui::TextColored({1, 0, 0, 1}, "%sLink %i: %s", std::string(indent, ' ').c_str(), idx,
                             links[idx]->getName().c_str());
        } else {
          if (ImGui::Selectable((std::string(indent, ' ') + "Link " + std::to_string(idx) + ": " +
                                 links[idx]->getName())
                                    .c_str())) {
            mSelect = true;
            mSelectedId = links[idx]->getId();
          }
        }
        for (uint32_t c : nodes[idx].children) {
          stack.push_back(c);
          indents.push_back(indent + 2);
        }
      }
      ImGui::TreePop();
    }
  }
}

void HudWorld::draw(SScene *scene, physx_id_t selectedId) {
  mSelect = false;
  if (ImGui::CollapsingHeader("World")) {
    ImGui::Text("Scene: %s", scene->getName().c_str());
    if (ImGui::TreeNode("Actors")) {
      auto actors = scene->getAllActors();
      for (uint32_t i = 0; i < actors.size(); ++i) {
        std::string name = actors[i]->getName();
        if (name.empty()) {
          name = "(no name)";
        }
        if (actors[i]->getId() == selectedId) {
          ImGui::TextColored({1, 0, 0, 1}, "%s", name.c_str());
        } else {
          if (ImGui::Selectable((name + "##actor" + std::to_string(i)).c_str())) {
            mSelect = true;
            mSelectedId = actors[i]->getId();
          }
        }
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Articulations")) {
      auto articulations = scene->getAllArticulations();
      for (uint32_t i = 0; i < articulations.size(); ++i) {
        std::string name = articulations[i]->getName();
        if (name.empty()) {
          name = "(no name)";
        }
        if (ImGui::TreeNode((name + "##articulation" + std::to_string(i)).c_str())) {
          auto links = articulations[i]->getBaseLinks();
          for (uint32_t j = 0; j < links.size(); ++j) {
            std::string name = links[j]->getName();
            if (name.empty()) {
              name = "(no name)";
            }
            if (links[j]->getId() == selectedId) {
              ImGui::TextColored({1, 0, 0, 1}, "%s", name.c_str());
            } else {
              if (ImGui::Selectable(
                      (name + "##a" + std::to_string(i) + "_" + std::to_string(j)).c_str())) {
                mSelect = true;
                mSelectedId = links[j]->getId();
              }
            }
          }
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Contacts")) {
      auto contacts = scene->getContacts();
      ImGui::Text("%ld contacts", contacts.size());

      for (uint32_t i = 0; i < contacts.size(); ++i) {
        if (ImGui::TreeNode((contacts[i]->actors[0]->getName() + "-" +
                             contacts[i]->actors[1]->getName() + "##contact" + std::to_string(i))
                                .c_str())) {
          if (ImGui::Selectable((contacts[i]->actors[0]->getName()).c_str())) {
            mSelect = true;
            mSelectedId = contacts[i]->actors[0]->getId();
          }
          if (ImGui::Selectable((contacts[i]->actors[1]->getName()).c_str())) {
            mSelect = true;
            mSelectedId = contacts[i]->actors[1]->getId();
          }
          if (contacts[i]->starts) {
            ImGui::Text("Type: start");
          } else if (contacts[i]->persists) {
            ImGui::Text("Type: persist");
          } else if (contacts[i]->ends) {
            ImGui::Text("Type: ends");
          }

          for (uint32_t j = 0; j < contacts[i]->points.size(); ++j) {
            auto &point = contacts[i]->points[j];
            ImGui::Text("Contact point %d", j);
            ImGui::Text("Separation: %.4g", point.separation);
            ImGui::Text("Impulse: %.4g %.4g %.4g", point.impulse.x, point.impulse.y, point.impulse.z);
          }

          ImGui::TreePop();
        }
      }

      ImGui::TreePop();
    }
  }
}

void HudCamera::draw(SScene *scene) {
  auto cameras = scene->getMountedCameras();
  auto actors = scene->getMountedActors();
  if (mIndex > cameras.size() + 1) {
    mIndex = 0;
  }

  if (ImGui::CollapsingHeader("Cameras")) {
    mRequestCameraView = false;
    ImGui::RadioButton("None", &mIndex, 0);
    for (uint32_t i = 0; i < cameras.size(); ++i) {
      ImGui::RadioButton((cameras[i]->getName() + "##camera" + std::to_string(i)).c_str(), &mIndex,
                         i + 1);
      if (mIndex == i + 1) {
        float near = cameras[i]->getNear();
        float far = cameras[i]->getFar();
        float fovy = cameras[i]->getFovy();
        uint32_t width = cameras[i]->getWidth();
        uint32_t height = cameras[i]->getHeight();
        ImGui::Text("Size: %d x %d", width, height);
        ImGui::Text("Range: %g - %g", near, far);
        ImGui::Text("Fovy(rad): %g", fovy);
        if (ImGui::Button("Camera View")) {
          mRequestCameraView = true;
          mRequestIndex = i + 1;
        }
      }
    }
  }
}

void HudObjectWindow::draw(SScene *scene, physx_id_t selectedId) {
  if (!scene) {
    return;
  }
  auto actor = scene->findActorById(selectedId);
  SArticulationBase *articulation{};
  if (!actor) {
    actor = scene->findArticulationLinkById(selectedId);
    if (actor) {
      articulation = static_cast<SLinkBase *>(actor)->getArticulation();
    }
  }

  ImGui::Begin("Object");
  mHudSettings.draw(scene);
  mHudWorld.draw(scene, selectedId);
  if (actor) {
    mHudActor.draw(actor);
  }
  if (articulation) {
    mHudArticulation.draw(articulation, actor);
  }
  if (scene->getMountedCameras().size()) {
    mHudCamera.draw(scene);
  }
  ImGui::End();
}

} // namespace Renderer
} // namespace sapien
