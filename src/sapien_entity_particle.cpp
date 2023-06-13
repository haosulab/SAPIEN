#include "sapien/sapien_entity_particle.h"

namespace sapien {
SEntityParticle::SEntityParticle(SScene *scene, Renderer::IRenderPointBody *renderBody)
    : SEntity(scene), mRenderBody(renderBody) {}
} // namespace sapien
