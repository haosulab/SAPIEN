#include "sapien/sapien_entity_deformable.h"

namespace sapien {
SEntityDeformable::SEntityDeformable(SScene *scene, Renderer::IRenderBody *renderBody)
    : SEntity(scene), mRenderBody(renderBody) {}
} // namespace sapien
