#include "urdf/urdf_loader.h"
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;
namespace fs = std::experimental::filesystem;

int main() {
  Assimp::Exporter exporter;
  for (int i = 0; i < aiGetExportFormatCount(); ++i) {
    const aiExportFormatDesc *formatDesc = aiGetExportFormatDescription(i);
    std::cout << i << std::endl;
    std::cout << formatDesc->description << std::endl;
    std::cout << formatDesc->fileExtension << std::endl;
    std::cout << std::endl;
  }

  return 0;
}
