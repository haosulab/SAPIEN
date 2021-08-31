//
// Created by jet on 8/30/21.
//

#pragma once

namespace sapien::Renderer {
struct KuafuConfig {
  bool mUseViewer = false;
  size_t mWidth = 640;
  size_t mHeight = 640;

  size_t mMaxGeometry = 256;
  size_t mMaxGeometryInstances = 256;
  size_t mMaxTextures = 256;

  std::string mAssetsPath;

  glm::vec4 mClearColor = glm::vec4(0.F, 0.F, 0.F, 1.F);

  uint32_t mPerPixelSampleRate = 64;
  bool mAccumulateFrames = false;
//
//  uint32_t mMaxPathDepth = 10;
//  uint32_t mPathDepth = 5;
//  uint32_t mRussianRouletteMinBounces = 3;
//
//  bool mUseDenoiser = false;  // todo
//
//  bool mNextEventEstimation = false;
//  uint32_t mNextEventEstimationMinBounces = 0; // temporary for debugging
//
//  float mVariance = 0.0F;
//  bool mUpdateVariance = false;
//
//  bool mRussianRoulette = true;


  std::shared_ptr<kuafu::Config> generate() {
    auto ret = std::make_shared<kuafu::Config>();
    ret->setGeometryLimit(mMaxGeometry);
    ret->setGeometryInstanceLimit(mMaxGeometryInstances);
    ret->setTextureLimit(mMaxTextures);
    ret->setAssetsPath(mAssetsPath);
    ret->setClearColor(mClearColor);
    ret->setPerPixelSampleRate(mPerPixelSampleRate);
    ret->setAccumulatingFrames(mAccumulateFrames);
    return ret;
  }
};
}
