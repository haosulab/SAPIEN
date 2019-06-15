#include "id_generator.h"

IDGenerator *IDGenerator::instance() {
  static IDGenerator *_instance;
  if (!_instance) {
    _instance = new IDGenerator();
  }
  return _instance;
}
