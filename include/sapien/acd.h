#pragma once
#include <memory>
#include <vector>

namespace sapien {
class SNonconvexMeshGeometry;
class SConvexMeshGeometry;

std::vector<std::shared_ptr<SConvexMeshGeometry>> CoACD(std::shared_ptr<SNonconvexMeshGeometry>);

}; // namespace sapien
