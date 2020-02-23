#include "sapien_articulation_base.h"
#include "sapien_link.h"

namespace sapien {

physx::PxTransform SArticulationBase::getRootPose() { return getRootLink()->getPose(); }

} // namespace sapien
