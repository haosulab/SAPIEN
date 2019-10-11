#include "id_generator.h"

namespace sapien {
IDGenerator *IDGenerator::RenderId() {
  static IDGenerator *_instance = nullptr;
  if (!_instance) {
    _instance = new IDGenerator();
  }
  return _instance;
}

IDGenerator *IDGenerator::LinkId() {
  static IDGenerator *_instance = nullptr;
  if (!_instance) {
    _instance = new IDGenerator();
  }
  return _instance;
}
}
