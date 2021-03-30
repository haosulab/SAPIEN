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
  if (mDestroyedState == 0) {
    mDestroyedState = 1;
    for (auto l : getBaseLinks()) {
      l->markDestroyed();
    }
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

std::string SArticulationBase::exportURDF() {
  std::string output = "<?xml version=\"1.0\"?>\n";
  output += "<robot name=\"\" >\n";

  auto *rootLink = getRootLink();
  output += exportTreeURDF(rootLink, PxTransform(PxIDENTITY()));

  output += "\n</robot>";
  return output;
}

static std::string exportLinkURDF(SLinkBase *link, physx::PxTransform extraTransform,
                                  bool returnVisual = false) {
  std::stringstream ss;
  std::string name = std::to_string(link->getIndex());

  auto mCollisionShapes = link->getCollisionShapes();
  for (auto mCollisionShape : mCollisionShapes) {
    ss << (returnVisual ? "<visual>" : "<collision>");

    PxTransform localPose = mCollisionShape->getLocalPose();
    PxTransform URDFPose = extraTransform * localPose; // TODO: check order
    Eigen::Quaternionf q;
    q.w() = URDFPose.q.w;
    q.x() = URDFPose.q.x;
    q.y() = URDFPose.q.y;
    q.z() = URDFPose.q.z;
    auto eulerAngles = q.toRotationMatrix().eulerAngles(2, 1, 0); // TODO: maybe simplified
    ss << "<origin xyz=\"" << URDFPose.p.x << " " << URDFPose.p.y << " " << URDFPose.p.z << "\" rpy=\""
       << eulerAngles[2] << " " << eulerAngles[1] << " " << eulerAngles[0] << "\" />";

    ss << "<geometry>";
    auto mPxShape = mCollisionShape->getPxShape();
    switch (mPxShape->getGeometryType()) {
    case PxGeometryType::eBOX: {
      PxBoxGeometry g;
      mPxShape->getBoxGeometry(g);
      auto x = g.halfExtents.x, y = g.halfExtents.y, z = g.halfExtents.z;
      x *= 2.0; y *= 2.0; z *= 2.0;
      ss << "<box size=\"" << x << " " << y << " " << z << "\" />";
      break;
    }
    case PxGeometryType::eSPHERE: {
      PxSphereGeometry g;
      mPxShape->getSphereGeometry(g);
      ss << "<sphere radius=\"" << g.radius << "\" />";
      break;
    }
    case PxGeometryType::eCAPSULE: {
      PxCapsuleGeometry g;
      mPxShape->getCapsuleGeometry(g);
      ss << "<capsule radius=\"" << g.radius << "\"length=\"" << g.halfHeight * 2.0 << "\" />";
      break;
    }
    default:
      std::cerr << "Currently not supported URDF geometry type: " << mPxShape->getGeometryType()
                << std::endl;
      exit(1);
      break;
    }
    ss << "</geometry>";
    ss << (returnVisual ? "</visual>" : "</collision>");
  }

  return ss.str();
}

static std::string exportJointURDF(SJointBase *joint, physx::PxTransform extraParentTransform,
                                   physx::PxTransform &extraChildTransform) {
  std::stringstream ss;

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

  PxTransform j2p = extraParentTransform * joint->getParentPose();
  PxTransform j2c = joint->getChildPose();
  PxTransform c2j = j2c.getInverse();

  Eigen::Quaternionf q;
  q.w() = j2p.q.w;
  q.x() = j2p.q.x;
  q.y() = j2p.q.y;
  q.z() = j2p.q.z;
  auto pEulerAngles = q.toRotationMatrix().eulerAngles(2, 1, 0); // TODO: maybe simplified
  extraChildTransform = c2j;

  ss << "<joint name=\"joint_" << joint->getChildLink()->getIndex() << "\" type=\"" << type
     << "\">";
  ss << "<origin xyz=\"" << j2p.p.x << " " << j2p.p.y << " " << j2p.p.z << "\" rpy=\""
     << pEulerAngles[2] << " " << pEulerAngles[1] << " " << pEulerAngles[0] << "\" />";
  ss << "<parent link=\"link_" << joint->getParentLink()->getIndex() << "\" />";
  ss << "<child link=\"link_" << joint->getChildLink()->getIndex() << "\" />";
  if (type == "prismatic" || type == "revolute") {
    ss << "<limit effort=\"0\" velocity=\"0\" lower=\"" << joint->getLimits()[0][0]
       << "\" upper=\"" << joint->getLimits()[0][1] << "\" />";
  }
  ss << "</joint>";

  return ss.str();
}

std::string SArticulationBase::exportTreeURDF(SLinkBase *link, physx::PxTransform extraTransform,
                                              bool exportVisual) {
  std::stringstream ss;
  std::string name = std::to_string(link->getIndex());

  /* Link */
  ss << "<link name=\"link_" << link->getIndex() << "\">";
  ss << exportLinkURDF(link, extraTransform); // collision
  if (exportVisual)
    ss << exportLinkURDF(link, extraTransform, true); // visual
  ss << "</link>";

  /* Joint */
  for (auto joint : getBaseJoints()) {
    if (joint->getParentLink() == link) {
      PxTransform extraChildTransform;
      ss << exportJointURDF(joint, extraTransform, extraChildTransform);
      ss << exportTreeURDF(joint->getChildLink(), extraChildTransform, exportVisual); // recursive
    }
  }

  return ss.str();
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
