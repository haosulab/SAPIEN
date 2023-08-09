#pragma once
#include "light_component.h"
#include "render_body_component.h"

namespace sapien {
namespace component {

std::vector<std::shared_ptr<Entity>> LoadScene(std::string const &filename);

}
} // namespace sapien
