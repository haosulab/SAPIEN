#pragma once
#include <Eigen/Eigen>

namespace sapien {

struct SceneConfig {
  Eigen::Vector3f gravity = {0, 0, -9.81}; // default gravity
  float static_friction = 0.3f;            // default static friction coefficient
  float dynamic_friction = 0.3f;           // default dynamic friction coefficient
  float restitution = 0.1f;                // default restitution coefficient
  float bounceThreshold = 2.f;             // relative velocity below this will not bounce
  float sleepThreshold = 0.005f;           // put to sleep if (kinetic energy/(mass) falls below
  float contactOffset = 0.01f;             // how close should contacts be generated
  uint32_t solverIterations = 10;          // solver position iterations, helps reduce jittering
  uint32_t solverVelocityIterations = 1;   // solver velocity iterations
  bool enablePCM = false;                  // Use persistent contact manifold solver for contact
  bool enableTGS = false;                  // use TGS solver
  bool enableCCD = false;                  // use continuous collision detection
  bool enableEnhancedDeterminism = false;  // improve determinism
  bool enableFrictionEveryIteration =
      true;                         // better friction calculation, recommended for robotics
  bool disableCollisionVisual = false;   // do not create visual shapes for collisions

  std::tuple<Eigen::Vector3f, float, float, float, float, float, float, uint32_t, uint32_t, bool, bool, bool, bool, bool, bool> getState() const {
    return std::make_tuple(
      gravity,
      static_friction,
      dynamic_friction,
      restitution,
      bounceThreshold,
      sleepThreshold,
      contactOffset,
      solverIterations,
      solverVelocityIterations,
      enablePCM,
      enableTGS,
      enableCCD,
      enableEnhancedDeterminism,
      enableFrictionEveryIteration,
      disableCollisionVisual
    );
  }

  void setState(const std::tuple<Eigen::Vector3f, float, float, float, float, float, float, uint32_t, uint32_t, bool, bool, bool, bool, bool, bool>& state) {
    gravity = std::get<0>(state);
    static_friction = std::get<1>(state);
    dynamic_friction = std::get<2>(state);
    restitution = std::get<3>(state);
    bounceThreshold = std::get<4>(state);
    sleepThreshold = std::get<5>(state);
    contactOffset = std::get<6>(state);
    solverIterations = std::get<7>(state);
    solverVelocityIterations = std::get<8>(state);
    enablePCM = std::get<9>(state);
    enableTGS = std::get<10>(state);
    enableCCD = std::get<11>(state);
    enableEnhancedDeterminism = std::get<12>(state);
    enableFrictionEveryIteration = std::get<13>(state);
    disableCollisionVisual = std::get<14>(state);
  }
};
} // namespace sapien
