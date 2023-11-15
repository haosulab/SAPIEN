#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>

namespace sapien {
namespace component {

/** System controls entity components in a scene.
 *  Components should register themselves to a system to enable lifecycle methods */
class System {
public:
  virtual ~System();
  virtual void step() = 0;
  virtual std::string getName() const = 0;

  template <class Archive> void save(Archive &archive) const {
    throw std::runtime_error("cereal workaround. should never be called.");
  }
  template <class Archive> void load(Archive &archive) {
    throw std::runtime_error("cereal workaround. should never be called.");
  }
};

} // namespace component
} // namespace sapien
