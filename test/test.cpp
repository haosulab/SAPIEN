#include "urdf/urdf_loader.h"
#include <fstream>
#include <iostream>
#include <sstream>

int main() {
  URDFLoader loader(nullptr);
  loader.load("../assets/urdf/test.urdf");
}
