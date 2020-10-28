#include "sapien_articulation_base.h"
#include "sapien_joint.h"
#include "sapien_link.h"
#include "sapien_scene.h"
#include <eigen3/Eigen/Eigen>
#ifdef _USE_PINOCCHIO
#include "pinocchio_model.h"
#endif

namespace sapien {

physx::PxTransform SArticulationBase::getRootPose() { return getRootLink()->getPose(); }

void SArticulationBase::markDestroyed() {
  for (auto l : getBaseLinks()) {
    l->markDestroyed();
  }
}

static std::string exportLink(SLinkBase *link) {
  std::stringstream ss;
  std::string name = std::to_string(link->getIndex());

  PxReal mass = link->getMass();
  PxVec3 inertia = link->getInertia();
  PxTransform massPose = link->getCMassLocalPose();

  Eigen::Quaternionf eq;
  eq.w() = massPose.q.w;
  eq.x() = massPose.q.x;
  eq.y() = massPose.q.y;
  eq.z() = massPose.q.z;
  auto angles = eq.toRotationMatrix().eulerAngles(2, 1, 0);

  ss << "<link name=\"link_" << link->getIndex() << "\">";
  ss << "<inertial>";

  ss << "<origin xyz=\"" << massPose.p.x << " " << massPose.p.y << " " << massPose.p.z
     << "\" rpy=\" " << angles[2] << " " << angles[1] << " " << angles[0] << "\" />";

  ss << "<mass value=\"" << mass << "\" />";

  ss << "<inertia ixx=\"" << inertia.x << "\" iyy=\"" << inertia.y << "\" izz=\"" << inertia.z
     << "\" ixy=\"0\" ixz=\"0\" iyz=\"0\" />";

  ss << "</inertial>";
  ss << "</link>";

  return ss.str();
}

static std::string exportJoint(SJointBase *joint, bool rootFixed) {
  std::stringstream ss;
  std::string name = "joint_" + std::to_string(joint->getChildLink()->getIndex());
  if (!joint->getParentLink()) {
    std::string type = rootFixed ? "fixed" : "floating";
    ss << "<joint name=\"" << name << "\" type=\"" << type << "\">";
    ss << "<parent link=\"world\" />";
    ss << "<child link=\"link_" << joint->getChildLink()->getIndex() << "\" />";
    ss << "</joint>";
    return ss.str();
  }
  std::string type;
  switch (joint->getType()) {
  case PxArticulationJointType::eFIX:
    type = "fixed";
    break;
  case PxArticulationJointType::ePRISMATIC:
    type = "prismatic";
    break;
  case PxArticulationJointType::eREVOLUTE:
    if (joint->getLimits()[0][0] < -10) {
      type = "continuous";
    } else {
      type = "revolute";
    }
    break;
  default:
    throw std::runtime_error("unknown joint type");
  }

  PxTransform j2p = joint->getParentPose();
  // PxTransform j2c = joint->getChildPose();
  PxTransform c2j = joint->getChildPose().getInverse();
  // PxTransform c2p = j2p * j2c.getInverse();
  // PxVec3 jxInC = j2c.q.rotate({1, 0, 0});

  Eigen::Quaternionf eq;
  eq.w() = j2p.q.w;
  eq.x() = j2p.q.x;
  eq.y() = j2p.q.y;
  eq.z() = j2p.q.z;
  auto angles = eq.toRotationMatrix().eulerAngles(2, 1, 0);

  // dummy link is the joint frame
  ss << "<link name=\"link_dummy_" << joint->getChildLink()->getIndex() << "\" />";

  // joint that connects parent with dummy
  ss << "<joint name=\"" << name << "\" type=\"" << type << "\">";
  ss << "<origin xyz=\"" << j2p.p.x << " " << j2p.p.y << " " << j2p.p.z << "\" rpy=\"" << angles[2]
     << " " << angles[1] << " " << angles[0] << "\" />";
  ss << "<axis xyz=\"1 0 0\" />";
  ss << "<parent link=\"link_" << joint->getParentLink()->getIndex() << "\" />";
  ss << "<child link=\"link_dummy_" << joint->getChildLink()->getIndex() << "\" />";
  if (type == "prismatic" || type == "revolute") {
    ss << "<limit effort=\"0\" velocity=\"0\" lower=\"" << joint->getLimits()[0][0]
       << "\" upper=\"" << joint->getLimits()[0][1] << "\" />";
  }
  ss << "</joint>";

  // fixed joint that connects dummy and child
  ss << "<joint name=\"joint_dummy_" << joint->getChildLink()->getIndex() << "\" type=\"fixed\">";

  eq.w() = c2j.q.w;
  eq.x() = c2j.q.x;
  eq.y() = c2j.q.y;
  eq.z() = c2j.q.z;
  angles = eq.toRotationMatrix().eulerAngles(2, 1, 0);

  ss << "<origin xyz=\"" << c2j.p.x << " " << c2j.p.y << " " << c2j.p.z << "\" rpy=\"" << angles[2]
     << " " << angles[1] << " " << angles[0] << "\" />";
  ss << "<axis xyz=\"0 0 0\" />";
  ss << "<parent link=\"link_dummy_" << joint->getChildLink()->getIndex() << "\" />";
  ss << "<child link=\"link_" << joint->getChildLink()->getIndex() << "\" />";
  ss << "</joint>";

  return ss.str();
}

std::string SArticulationBase::exportKinematicsChainAsURDF(bool fixRoot) {
  std::string output = "<?xml version=\"1.0\"?>\n<robot name=\"\" >";
  output += "<link name=\"world\" />";
  struct LinkNode {
    std::string name;
  };
  for (SLinkBase *l : getBaseLinks()) {
    output += exportLink(l);
  }
  for (SJointBase *j : getBaseJoints()) {
    output += exportJoint(j, fixRoot);
  }

  output += "</robot>";
  return output;
}

#ifdef _USE_PINOCCHIO
std::unique_ptr<PinocchioModel> SArticulationBase::createPinocchioModel() {
  PxVec3 gravity = getScene()->getPxScene()->getGravity();
  auto pm = PinocchioModel::fromURDFXML(exportKinematicsChainAsURDF(true),
                                        {gravity.x, gravity.y, gravity.z});
  std::vector<std::string> jointNames;
  std::vector<std::string> linkNames;
  for (auto j : getBaseJoints()) {
    if (j->getDof() > 0) {
      jointNames.push_back("joint_" + std::to_string(j->getChildLink()->getIndex()));
    }
  }
  for (auto l : getBaseLinks()) {
    linkNames.push_back("link_" + std::to_string(l->getIndex()));
  }
  pm->setJointOrder(jointNames);
  pm->setLinkOrder(linkNames);
  return pm;
}
#endif

} // namespace sapien
