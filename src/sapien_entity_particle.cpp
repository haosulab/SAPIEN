#include "sapien_entity_particle.h"

namespace sapien {
SEntityParticle::SEntityParticle(SScene *scene, Renderer::IPxrPointBody *renderBody)
    : SEntity(scene), mRenderBody(renderBody) {}
} // namespace sapien
