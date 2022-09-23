#pragma once
#include <eigen3/Eigen/Eigen>

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
  bool enableAdaptiveForce = false; // improve solver convergence
  bool disableCollisionVisual = false;   // do not create visual shapes for collisions
};
} // namespace sapien
