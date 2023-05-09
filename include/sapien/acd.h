#pragma once
#include <memory>
#include <vector>

namespace sapien {
class SNonconvexMeshGeometry;
class SConvexMeshGeometry;

std::vector<std::shared_ptr<SConvexMeshGeometry>>
CoACD(std::shared_ptr<SNonconvexMeshGeometry> g,
      double threshold = 0.05,        // concavity stop criterion (0.05)
      bool preprocess = true,         // run manifold algorithm as preprocess (true)
      int preprocess_resolution = 30, // preprocess resolution (30)
      bool pca = false,               // PCA (false)
      bool merge = true,              // merge convex hull after decomposition
      int mcts_max_depth = 3, int mcts_nodes = 20, int mcts_iteration = 150,
      unsigned int seed = 0);

std::shared_ptr<SConvexMeshGeometry> Remesh(std::shared_ptr<SNonconvexMeshGeometry> g,
                                            int resolution = 30, double level_set = 0.55);

void coacd_set_log_level(std::string_view level);

}; // namespace sapien
