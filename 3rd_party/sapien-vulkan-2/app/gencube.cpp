#include "svulkan2/common/fs.h"
#include "svulkan2/core/context.h"
#include "svulkan2/renderer/renderer.h"
#include "svulkan2/scene/scene.h"
#include "svulkan2/shader/compute.h"
#include "svulkan2/shader/shader.h"
#include "svulkan2/ui/ui.h"
#include <iostream>

using namespace svulkan2;

int main(int argc, char *argv[]) {
  if (argc != 7) {
    std::cerr << "gencube takes exactly 6 filenames" << std::endl;
    return 1;
  }

  auto context = svulkan2::core::Context::Create(true, 5000, 5000, 4);
  auto manager = context->createResourceManager();
  auto cubemap = context->getResourceManager()->CreateCubemapFromFiles(
      {argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]}, 5);
  cubemap->load();
  cubemap->uploadToDevice();
  cubemap->exportKTX("out.ktx");

  return 0;
}
