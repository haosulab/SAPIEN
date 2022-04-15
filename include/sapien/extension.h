#pragma once
#include <memory>


namespace sapien {

class  SceneMultistepCallback {
public:
  // virtual void beforeMultistep() = 0;
  // virtual void beforeStep(int step) = 0;
  // virtual void afterStep(int step) = 0;
  // virtual void afterMultistep() = 0;
  virtual void beforeMultistep(){};
  virtual void beforeStep(int step){};
  virtual void afterStep(int step){};
  virtual void afterMultistep(){};

  SceneMultistepCallback(){};
  virtual ~SceneMultistepCallback() = default;
};

}; // namespace sapien
