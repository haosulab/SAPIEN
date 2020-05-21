#include "sapien_articulation_base.h"
#include "sapien_joint.h"
#include "sapien_link.h"
#include <eigen3/Eigen/Eigen>

namespace sapien {

physx::PxTransform SArticulationBase::getRootPose() { return getRootLink()->getPose(); }

static std::string exportLink(SLinkBase *link) {
  std::stringstream ss;
  std::string name = std::to_string(link->getIndex());

  PxReal mass = link->getMass();
  PxVec3 inertia = link->getInertia();
  PxTransform massPose = link->getCMassLocalPose();

  auto angles = Eigen::Quaternionf(massPose.q.w, massPose.q.x, massPose.q.y, massPose.q.z)
                    .toRotationMatrix()
                    .eulerAngles(0, 1, 2);

  ss << "<link name=\"link_" << link->getIndex() << "\">";
  ss << "<inertial>";

  ss << "<origin xyz=\"" << massPose.p.x << " " << massPose.p.y << " " << massPose.p.z
     << "\" rpy=\" " << angles[0] << " " << angles[1] << " " << angles[2] << "\" />";

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
  PxTransform j2c = joint->getChildPose();
  PxTransform c2p = j2p * j2c.getInverse();
  PxVec3 jxInC = j2c.q.rotate({1, 0, 0});

  auto angles = Eigen::Quaternionf(c2p.q.w, c2p.q.x, c2p.q.y, c2p.q.z)
                .toRotationMatrix()
                .eulerAngles(0, 1, 2);

  ss << "<joint name=\"" << name << "\" type=\"" << type << "\">";

  ss << "<origin xyz=\""
     << c2p.p.x << " " << c2p.p.y << " " << c2p.p.z
     << "\" rpy=\""
     << angles[0] << " " << angles[1] << " " << angles[2]
     << "\" />";

  ss << "<axis xyz=\"" << jxInC.x << " " << jxInC.y << " " << jxInC.z <<  "\" />";

  ss << "<parent link=\"link_" << joint->getParentLink()->getIndex() << "\" />";
  ss << "<child link=\"link_" << joint->getChildLink()->getIndex() << "\" />";
  if (type == "prismatic" || type == "revolute") {
    ss << "<limit effort=\"0\" velocity=\"0\" lower=\""
       << joint->getLimits()[0][0] << "\" upper=\""
       << joint->getLimits()[0][1] << "\" />";
  }
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

} // namespace sapien
