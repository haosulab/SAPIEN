#include "id_generator.h"

namespace sapien {
IDGenerator *IDGenerator::instance() {
  static IDGenerator *_instance;
  if (!_instance) {
    _instance = new IDGenerator();
  }
  return _instance;
}

}
