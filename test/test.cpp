#include "urdf/urdf_loader.h"
#include <fstream>
#include <iostream>
#include <sstream>

int main() {
  URDFUtil::URDFLoader loader;
  loader.load("../assets/urdf/test.urdf");
}
