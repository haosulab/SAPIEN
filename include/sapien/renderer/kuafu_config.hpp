//
// Created by jet on 8/30/21.
//

#pragma once
#include <spdlog/spdlog.h>

namespace sapien::Renderer {
struct KuafuConfig {
  bool mUseViewer = false;
  size_t mViewerWidth = 640;
  size_t mViewerHeight = 640;

  size_t mMaxGeometry = 1024;
  size_t mMaxGeometryInstances = 1024;
  size_t mMaxTextures = 1024;
  size_t mMaxMaterials = 1024;

  std::string mAssetsPath;

  uint32_t mPerPixelSampleRate = 32;
  bool mAccumulateFrames = false;

  uint32_t mPathDepth = 8;
  bool mUseDenoiser = false;

//  uint32_t mMaxPathDepth = 10;
//  bool mRussianRoulette = true;
//  uint32_t mRussianRouletteMinBounces = 3;

  std::shared_ptr<kuafu::Config> generate() {
    auto ret = std::make_shared<kuafu::Config>();
    ret->setInitialWidth(mViewerWidth);
    ret->setInitialHeight(mViewerHeight);
    ret->setGeometryLimit(mMaxGeometry);
    ret->setGeometryInstanceLimit(mMaxGeometryInstances);
    ret->setTextureLimit(mMaxTextures);
    ret->setMaterialLimit(mMaxMaterials);
    ret->setAssetsPath(mAssetsPath);
    ret->setPerPixelSampleRate(mPerPixelSampleRate);
    ret->setAccumulatingFrames(mAccumulateFrames);
    ret->setPathDepth(mPathDepth);
    ret->setUseDenoiser(mUseDenoiser);
    ret->setPresent(mUseViewer);
    return ret;
  }
};
}
