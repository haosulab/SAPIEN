#include "hud_object_window.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_articulation_base.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "articulation/sapien_kinematic_joint.h"
#include "articulation/sapien_link.h"
#include "sapien_actor.h"
#include "sapien_actor_base.h"
#include "sapien_scene.h"
#include "sapien_contact.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/norm.hpp>

namespace sapien {
namespace Renderer {

static void PrintPose(PxTransform const &pose) {
  glm::vec3 angles =
      glm::eulerAngles(glm::quat(pose.q.w, pose.q.x, pose.q.y, pose.q.z)) /
      glm::pi<float>() * 180.f;

  ImGui::Text("Position: %.4g %.4g %.4g", pose.p.x, pose.p.y, pose.p.z);
  ImGui::Text("Quat (wxyz): %.4g %.4g %.4g %.4g", pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  ImGui::Text("Euler (degree): %.4g %.4g %.4g", angles.x, angles.y, angles.z);
};


void HudActor::draw(SActorBase *actor) {
  if (ImGui::CollapsingHeader("Actor")) {
    // name
    ImGui::Text("Name: %s", actor->getName().c_str());

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
      auto &contacts = scene->getContacts();
      ImGui::Text("%ld contacts", contacts.size());
      for (uint32_t i = 0; i < contacts.size(); ++i) {
        if (ImGui::TreeNode((contacts[i].actors[0]->getName() + "-" +
                             contacts[i].actors[1]->getName() + "##contact" +
                             std::to_string(i))
                            .c_str())) {
          if (ImGui::Selectable((contacts[i].actors[0]->getName()).c_str())) {
            mSelect = true;
            mSelectedId = contacts[i].actors[0]->getId();
          }
          if (ImGui::Selectable((contacts[i].actors[1]->getName()).c_str())) {
            mSelect = true;
            mSelectedId = contacts[i].actors[1]->getId();
          }
          ImGui::Text("Separation: %.4f", contacts[i].separation);
          ImGui::Text("Impulse: %.4f %.4f %.4f", contacts[i].impulse.x,
                      contacts[i].impulse.y, contacts[i].impulse.z);
          if (contacts[i].starts) {
            ImGui::Text("Type: start");
          } else if (contacts[i].persists) {
            ImGui::Text("Type: persist");
          } else if (contacts[i].ends) {
            ImGui::Text("Type: ends");
          }
          if (ImGui::Selectable("Position")) {
            mSelect = true;
            mSelectedId = 0;
            // currentScene->getScene()->clearAxes();
            glm::vec3 v = {contacts[i].normal.x, contacts[i].normal.y, contacts[i].normal.z};
            glm::vec3 t1 = glm::cross(v, glm::vec3(1, 0, 0));
            if (glm::length2(t1) < 0.01) {
              t1 = glm::cross(v, glm::vec3(0, 1, 0));
            }
            t1 = glm::normalize(t1);
            glm::vec3 t2 = glm::cross(v, t1);
            glm::mat3 m(v, t1, t2);

            // currentScene->getScene()->addAxes(
            //     {contacts[i].point.x, contacts[i].point.y, contacts[i].point.z},
            //     glm::quat(m));
          }
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
  }
}

void HudObjectWindow::draw(SScene *scene, physx_id_t selectedId) {
  if (!scene) {
    return;
  }
  auto actor = scene->findActorById(selectedId);
  if (!actor) {
    actor = scene->findArticulationLinkById(selectedId);
  }

  ImGui::Begin("Object");
  mHudWorld.draw(scene, selectedId);
  if (actor) {
    mHudActor.draw(actor);
  }
  ImGui::End();
}

} // namespace Renderer
} // namespace sapien
